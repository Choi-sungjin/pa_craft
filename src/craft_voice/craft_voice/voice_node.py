"""voice_node: camera + planning state machine -> GPT -> speech.

Narrates what the robot is doing and what it sees, when its action changes and
every narrate_period_s while it works.
sub: /planning/state (std_msgs/String, transient local), /planning/diagnostics,
     /control/status (craft_interfaces/ControlStatus, optional), context_topics (String),
     what perception sees: odom_topic (nav_msgs/Odometry), objects_topic
     (craft_perception/DetectionArray), pucks_topic (PuckObservationArray), tags_topic (Int32MultiArray),
     camera: /vision/d435/frame_meta + /dev/shm/craft_vision (camera:=d435),
             /vision/c310/frame_meta + /dev/shm/craft_c310 (camera:=c310),
             image_topic (camera:=topic, raw or compressed), or nothing (camera:=none)
     /voice/ask (String: a question), /voice/say (String: speak as is),
     /voice/command (String: describe | status | mute | unmute | stop)
pub: /voice/utterance (String "kind: text", every sentence before it is spoken),
     /voice/speaking (Bool), /voice/diagnostics (DiagnosticArray, 1 Hz)
Needs OPENAI_API_KEY in the environment; without it state changes are spoken
from fixed templates through fallback_tts_command (or only published).
"""

from __future__ import annotations

import math
import os
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String, UInt64MultiArray

from .audio import CommandPlayer, CommandTts, split_command
from .frames import ShmFrameReader, compressed_to_bgr, image_to_bgr
from .narrator import Narrator, NarratorConfig
from .robot_state import StateTracker, parse_list, parse_mapping

SHM_CAMERAS = {  # camera -> (shm name, has depth, frame_meta topic)
    'd435': ('/craft_vision', True, '/vision/d435/frame_meta'),
    'c310': ('/craft_c310', False, '/vision/c310/frame_meta'),
}

PARAMETERS = {
    'camera': 'd435',
    'image_topic': '/camera/image_raw',
    'image_transport': 'raw',
    'camera_description': '로봇 앞쪽 D435 카메라',
    'use_image': True,
    'frame_max_age_s': 2.0,
    'image_max_side': 512,
    'jpeg_quality': 80,
    'planning_state_topic': '/planning/state',
    'planning_diagnostics_topic': '/planning/diagnostics',
    'control_status_topic': '/control/status',
    'context_topics': '/perception/localization/state',
    'announce_states': 'Ready,Run.Execute,Run.Complete,Run.Stopping,Fault',
    'step_names': '',
    'state_cooldown_s': 10.0,
    'state_mode': 'gpt',
    'fault_reason_wait_s': 1.2,
    'narrate_period_s': 20.0,
    'narrate_idle_period_s': 0.0,
    'odom_topic': '/perception/state/odom',
    'objects_topic': '/perception/vision/d435/detections',
    'pucks_topic': '/perception/vision/d435/puck_observations',
    'tags_topic': '/perception/vision/d435/apriltag',
    'min_interval_s': 5.0,
    'max_gpt_calls_per_min': 12,
    'repeat_window_s': 30.0,
    'interrupt_on_fault': True,
    'language': 'ko',
    'persona': '',
    'max_chars': 150,
    'model': 'gpt-5.5',
    'reasoning_effort': 'low',
    'image_detail': 'low',
    'max_output_tokens': 400,
    'tts_model': 'gpt-4o-mini-tts',
    'tts_voice': 'coral',
    'tts_instructions': '밝고 또렷한 한국어 안내 로봇 목소리로, 조금 빠르게 말해.',
    'tts_speed': 1.0,
    'tts_sample_rate': 24000,
    'api_base_url': '',
    'request_timeout_s': 15.0,
    'fault_tts_timeout_s': 4.0,
    'gpt_state_deadline_s': 6.0,
    'gpt_narrate_deadline_s': 10.0,
    'gpt_ask_deadline_s': 15.0,
    'tts_deadline_s': 8.0,
    'player_command': 'aplay -q {file}',
    'fallback_tts_command': '',
}


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in PARAMETERS}
        log = self.get_logger()

        tracker = StateTracker(announce=tuple(parse_list(p['announce_states'])), cooldown_s=p['state_cooldown_s'],
                               fault_reason_wait_s=p['fault_reason_wait_s'],
                               step_names=parse_mapping(p['step_names']))
        config = NarratorConfig(
            language=p['language'], persona=p['persona'], camera_description=p['camera_description'],
            max_chars=p['max_chars'], state_mode=p['state_mode'], narrate_period_s=p['narrate_period_s'],
            narrate_idle_period_s=p['narrate_idle_period_s'],
            min_interval_s=p['min_interval_s'], max_gpt_calls_per_min=p['max_gpt_calls_per_min'],
            repeat_window_s=p['repeat_window_s'], use_image=p['use_image'] and p['camera'] != 'none',
            image_max_side=p['image_max_side'], jpeg_quality=p['jpeg_quality'],
            interrupt_on_fault=p['interrupt_on_fault'], fault_tts_deadline_s=p['fault_tts_timeout_s'] + 1.0,
            gpt_state_deadline_s=p['gpt_state_deadline_s'], tts_deadline_s=p['tts_deadline_s'],
            gpt_narrate_deadline_s=p['gpt_narrate_deadline_s'], gpt_ask_deadline_s=p['gpt_ask_deadline_s'])
        if p['state_mode'] not in ('gpt', 'template'):
            raise ValueError("state_mode must be 'gpt' or 'template'")

        backend = None
        if os.environ.get('OPENAI_API_KEY'):
            try:
                from .gpt import OpenAIBackend
                backend = OpenAIBackend(
                    model=p['model'], tts_model=p['tts_model'], voice=p['tts_voice'],
                    tts_instructions=p['tts_instructions'], image_detail=p['image_detail'],
                    max_output_tokens=p['max_output_tokens'], reasoning_effort=p['reasoning_effort'],
                    tts_speed=p['tts_speed'], sample_rate=p['tts_sample_rate'],
                    timeout_s=p['request_timeout_s'], urgent_timeout_s=p['fault_tts_timeout_s'],
                    base_url=p['api_base_url'])
            except Exception as error:  # e.g. openai package missing
                log.error(f'OpenAI backend unavailable ({type(error).__name__}: {error}); templates only')
        else:
            log.warning('OPENAI_API_KEY is not set: no GPT and no OpenAI speech, templates only')
        fallback = split_command(p['fallback_tts_command'])
        player = split_command(p['player_command'])

        self._utterance_pub = self.create_publisher(String, '/voice/utterance', 10)
        self._speaking_pub = self.create_publisher(Bool, '/voice/speaking', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, '/voice/diagnostics', 10)
        self._frame_lock = threading.Lock()
        self._frame_max_age_s = p['frame_max_age_s']
        self._latest = None  # (receive time, message or meta)
        self._reader = None
        self._camera = p['camera']
        self._setup_camera(p)

        self.narrator = Narrator(
            config, tracker, backend=backend, player=CommandPlayer(player) if player else None,
            frame_provider=self._frame if config.use_image else None,
            fallback_tts=CommandTts(fallback) if fallback else None,
            on_text=self._publish_text, on_speaking=self._publish_speaking, logger=log)

        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)  # matches planning_master
        self.create_subscription(String, p['planning_state_topic'],
                                 lambda m: self.narrator.planning_state(m.data), latched)
        self.create_subscription(DiagnosticArray, p['planning_diagnostics_topic'], self._on_diagnostics, 10)
        try:
            from craft_interfaces.msg import ControlStatus
            self.create_subscription(ControlStatus, p['control_status_topic'], lambda m: self.narrator.control_status(
                m.ready, m.fault, m.held, m.stopped, m.reason), 10)
        except ImportError:
            log.warning('craft_interfaces not found: control status is left out of the context')
        for topic in parse_list(p['context_topics']):  # volatile: matches latched and plain publishers
            self.create_subscription(String, topic, lambda m, t=topic: self.narrator.context(t, m.data), 10)
        self._subscribe_perception(p)
        self.create_subscription(String, '/voice/ask', lambda m: self.narrator.ask(m.data), 10)
        self.create_subscription(String, '/voice/say', lambda m: self.narrator.say(m.data), 10)
        self.create_subscription(String, '/voice/command', self._on_command, 10)
        self.create_timer(0.1, self.narrator.tick)
        self.create_timer(1.0, self._publish_diagnostics)
        log.info(f"voice ready: camera={p['camera']} gpt={'on' if backend else 'off'} model={p['model']} "
                 f"tts={p['tts_model']}/{p['tts_voice']} state_mode={p['state_mode']} "
                 f"narrate every {p['narrate_period_s']}s player={' '.join(player) or 'none'}")

    # ---- what perception sees ---------------------------------------------------------------
    def _subscribe_perception(self, p):
        log = self.get_logger()
        if p['odom_topic']:
            try:
                from nav_msgs.msg import Odometry
                self.create_subscription(Odometry, p['odom_topic'], lambda m: self.narrator.motion(
                    m.twist.twist.linear.x, m.twist.twist.angular.z), qos_profile_sensor_data)
            except ImportError:
                log.warning('nav_msgs not found: motion is left out')
        if p['tags_topic']:
            from std_msgs.msg import Int32MultiArray
            self.create_subscription(Int32MultiArray, p['tags_topic'],
                                     lambda m: self.narrator.tags(list(m.data)), qos_profile_sensor_data)
        if not (p['objects_topic'] or p['pucks_topic']):
            return
        try:
            from craft_perception.msg import DetectionArray, PuckObservationArray
        except ImportError:
            log.warning('craft_perception messages not found: YOLO objects and pucks are left out')
            return
        if p['objects_topic']:
            self.create_subscription(DetectionArray, p['objects_topic'], lambda m: self.narrator.objects(
                [d.label for d in m.detections]), qos_profile_sensor_data)
        if p['pucks_topic']:
            self.create_subscription(PuckObservationArray, p['pucks_topic'], lambda m: self.narrator.pucks(
                # Without a depth axis the centre is (0,0,0) and 'standing' is meaningless: report unknown.
                [(o.standing, math.sqrt(o.center.x ** 2 + o.center.y ** 2 + o.center.z ** 2))
                 if o.axis_valid else (None, None) for o in m.observations]), qos_profile_sensor_data)

    # ---- camera -----------------------------------------------------------------------------
    def _setup_camera(self, p):
        camera = p['camera']
        if camera in SHM_CAMERAS:
            name, depth, topic = SHM_CAMERAS[camera]
            self._reader = ShmFrameReader(name, depth)
            self.create_subscription(UInt64MultiArray, topic, lambda m: self._store(list(m.data)),
                                     qos_profile_sensor_data)
        elif camera == 'topic':
            if p['image_transport'] == 'compressed':
                from sensor_msgs.msg import CompressedImage as ImageType
            elif p['image_transport'] == 'raw':
                from sensor_msgs.msg import Image as ImageType
            else:
                raise ValueError("image_transport must be 'raw' or 'compressed'")
            self.create_subscription(ImageType, p['image_topic'], self._store, qos_profile_sensor_data)
        elif camera != 'none':
            raise ValueError("camera must be d435, c310, topic or none")

    def _store(self, item):
        with self._frame_lock:
            self._latest = (time.monotonic(), item)

    def _frame(self):
        """Called from the think thread: newest frame as BGR, or None if there is none fresh."""
        for _ in range(3):
            with self._frame_lock:
                latest = self._latest
            if latest is None or time.monotonic() - latest[0] > self._frame_max_age_s:
                return None
            received, item = latest
            if self._reader is None:
                if hasattr(item, 'format'):
                    return compressed_to_bgr(item.data)
                return image_to_bgr(item.encoding, item.height, item.width, item.step, item.data)
            frame = self._reader.snapshot(item)
            if frame is not None:
                return frame.bgr
            time.sleep(0.04)  # slot busy or overwritten: take the next frame_meta
        return None

    # ---- callbacks / outputs ----------------------------------------------------------------
    def _on_diagnostics(self, msg: DiagnosticArray):
        for status in msg.status:
            self.narrator.diagnostics(status.name, int.from_bytes(status.level, 'little')
                                      if isinstance(status.level, bytes) else int(status.level),
                                      status.message, {kv.key: kv.value for kv in status.values})

    def _on_command(self, msg: String):
        self.get_logger().info(f'/voice/command: {self.narrator.command(msg.data)}')

    def _publish_text(self, kind: str, text: str):
        self._utterance_pub.publish(String(data=f'{kind}: {text}'))

    def _publish_speaking(self, speaking: bool):
        self._speaking_pub.publish(Bool(data=speaking))

    def _publish_diagnostics(self):
        stats = self.narrator.status()
        status = DiagnosticStatus(name='voice/narrator', hardware_id='voice_node',
                                  message='muted' if stats['muted'] else 'ok')
        status.level = DiagnosticStatus.OK
        status.values = [KeyValue(key=key, value=str(value)) for key, value in sorted(stats.items())]
        with self._frame_lock:
            age = time.monotonic() - self._latest[0] if self._latest else float('inf')
        status.values.append(KeyValue(key='frame_age_s', value=f'{age:.2f}'))
        msg = DiagnosticArray(status=[status])
        msg.header.stamp = self.get_clock().now().to_msg()
        self._diagnostics_pub.publish(msg)

    def destroy_node(self):
        self.narrator.close()
        if self._reader is not None:
            self._reader.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
