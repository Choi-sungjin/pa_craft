"""voice_node: camera + planning state machine -> GPT -> speech.

Narrates what the robot is doing and what it sees, when its action changes and
every narrate_period_s while it works.
sub: /planning/state (std_msgs/String, transient local), /planning/diagnostics,
     /control/status (craft_interfaces/ControlStatus, optional), context_topics (String),
     what perception sees: odom_topic (nav_msgs/Odometry), objects_topic
     (craft_perception/DetectionArray), pucks_topic (PuckObservationArray), tags_topic (Int32MultiArray),
     camera: /vision/d435/frame_meta + /dev/shm/craft_vision (camera:=d435),
             /vision/c310/frame_meta + /dev/shm/craft_c310 (camera:=c310),
             image_topic (camera:=topic, raw or compressed), or nothing (camera:=none);
     image_source:=input sends GPT the frame the state machine's vision input came from (same stamp as
     objects_topic / pucks_topic, see camera.py), latest the newest frame
     /voice/ask (String: a question), /voice/say (String: speak as is),
     /voice/command (String: describe | status | wake | mute | unmute | stop | listen | deaf)
     microphone (listen:=true): "크래프트" -> says what it is doing; "크래프트, <question>" -> answers
pub: /voice/utterance (String "kind: text", every sentence before it is spoken),
     /voice/heard (String: what was asked by voice), /voice/speaking (Bool),
     /voice/diagnostics (DiagnosticArray, 1 Hz)
Needs OPENAI_API_KEY in the environment; without it state changes are spoken
from fixed templates through fallback_tts_command (or only published).
"""

from __future__ import annotations

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.type_support import check_for_type_support
from rosidl_generator_py.import_type_support_impl import UnsupportedTypeSupport
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String, UInt64MultiArray

from .audio import CommandPlayer, CommandTts, StreamPlayer, split_command, stream_command_for
from .camera import CameraFrames, stamp_ns
from .clock import clock_unset
from .frames import ShmFrameReader
from .gpt import DEFAULT_STT_MODEL, DEFAULT_TTS_MODEL, DEFAULT_TTS_VOICE
from .mission import MISSION_CONTEXT, STEP_NAMES
from .narrator import Narrator, NarratorConfig
from .robot_state import StateTracker, parse_list, parse_mapping
from .wiring import DEFAULT_CACHE_DIR, DEFAULT_MIC_COMMAND, load_lines, load_reactions, make_cache, make_listener

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
    'image_source': 'input',
    'input_frame_interval_s': 0.2,
    'image_max_side': 512,
    'jpeg_quality': 80,
    'planning_state_topic': '/planning/state',
    'planning_diagnostics_topic': '/planning/diagnostics',
    'control_status_topic': '/control/status',
    'context_topics': '/perception/localization/state,/planning/box_phase,/planning/box_status',
    # std_msgs/String topics that are events (the last command, the mission): kept for half an hour, with their age
    'event_topics': '/planning/command,/planning/mission,/planning/approach_box,/planning/approach_box_near',
    # std_msgs/Bool, context; a grasp (gripper_grasped false -> true) is also said
    'flag_topics': '/planning/hold,/control/planning_held,/control/gripper_grasped',
    # ---- further signals ('' = not read): facts for answers, a few said at once (signals.py) ----
    'task_result_topic': '/control/task_result',            # craft_interfaces/TaskResult: what control did with the task
    'task_command_topic': '/planning/task_command',         # craft_interfaces/TaskCommand: what planning commanded
    'barcode_topic': '/perception/vision/uppercam/barcode',  # std_msgs/String, latched: "바코드 읽었어!"
    # craft_perception/TrafficLight, latched: "신호등 깜빡인다!", "신호등 다 셌어! 빨강 n번, ..."
    'traffic_light_topic': '/perception/vision/uppercam/traffic_light',
    'obstacles_topic': '/perception/lidar/obstacle',        # geometry_msgs/PoseArray: the nearest obstacle
    'box_topic': '/perception/box/observation',             # craft_interfaces/BoxObservation: "상자 찾았다!"
    'announce_states': 'Ready,Run.Execute,Run.Complete,Run.Stopping,Fault',
    'step_names': STEP_NAMES,
    'mission_context': MISSION_CONTEXT,
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
    'model': 'gpt-5.4-mini,gpt-6-luna,gpt-5.5',
    'reasoning_effort': 'none',
    'image_detail': 'low',
    'max_output_tokens': 400,
    'tts_model': DEFAULT_TTS_MODEL,
    'tts_voice': DEFAULT_TTS_VOICE,
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
    'tts_stream': True,
    'stream_player_command': '',
    'fallback_tts_command': '',
    # ---- persona, reactions, memory ----
    'style': 'polite',                      # polite | cute (short friendly 반말)
    'variety': False,                       # further wordings in turn, drives said with distance / count / time
    'menu_file': '',                        # cafeteria menu (config/menu.yaml): "오늘 점심 메뉴 뭐야?" ('' = off)
    'menu_items': 4,                        # dishes named in an answer
    'goal_topic': '/planning/target_base_pose',   # geometry_msgs/PoseStamped, map frame ('' = not read)
    'pose_topic': '/planning/odom',               # nav_msgs/Odometry, map frame ('' = not read)
    'phrases_file': '',                     # JSON {key: sentence} replacing single fixed sentences
    'announce_situations': 'replan,blocked',   # diagnostics waiting situations said at once ('clear', 'rerouted' too if wanted)
    'notice': True,                         # remarks on what it notices by itself
    # 혼잣말(chatter.py): 일하지 않고 서 있을 때 평균 이 간격으로 정해 둔 한마디(0 = 끔), 간격 흔들림 비율,
    # 마지막으로 말하거나 들은 뒤 이만큼은 조용히
    'chatter_period_s': 0.0,
    'chatter_jitter': 0.4,
    'chatter_quiet_s': 60.0,
    'reactions': '',                        # recordings: "replan=/p/a.wav,step_done=/p/b.wav,run_done=/p/c.wav,idle=..."
    'speech_cache_dir': DEFAULT_CACHE_DIR,  # fixed sentences are synthesized once and kept ('' = off)
    # ---- microphone: called by name ----
    'listen': False,
    'mic_command': DEFAULT_MIC_COMMAND,
    'wake_words': '크래프트',
    # 반응 단어: 들으면 대화 대신 녹음을 바로 튼다. "단어|별칭|별칭=녹음.wav;단어2=..."
    # 예) "젯슨|제슨|제트슨|jetson=~/craft-voice-sandbox/recordings/jetson_yeap.wav"
    'word_reactions': '',
    'wake_tolerance': 2,
    'wake_mode': 'think',                   # the name alone: think (GPT) | template (fixed words)
    'wake_ack': True,
    'gpt_voice_deadline_s': 5.0,
    'follow_up_s': 6.0,                     # after an answer, the next sentence needs no name (0 = off)
    'listen_tail_s': 0.6,
    'stt_model': DEFAULT_STT_MODEL,
    'stt_language': 'ko',
    'stt_prompt': '',
    'max_stt_per_min': 20,
    'vad_min_rms': 150.0,
    'vad_loud_ratio': 2.5,
}


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in PARAMETERS}
        log = self.get_logger()

        if p['style'] not in ('polite', 'cute'):
            raise ValueError("style must be 'polite' or 'cute'")
        tracker = StateTracker(announce=tuple(parse_list(p['announce_states'])), cooldown_s=p['state_cooldown_s'],
                               fault_reason_wait_s=p['fault_reason_wait_s'],
                               step_names=parse_mapping(p['step_names']), style=p['style'],
                               lines=load_lines(p['phrases_file'], log),
                               announce_situations=tuple(parse_list(p['announce_situations'])),
                               variety=p['variety'])
        config = NarratorConfig(
            language=p['language'], persona=p['persona'], camera_description=p['camera_description'],
            mission_context=p['mission_context'],
            max_chars=p['max_chars'], state_mode=p['state_mode'], narrate_period_s=p['narrate_period_s'],
            narrate_idle_period_s=p['narrate_idle_period_s'],
            min_interval_s=p['min_interval_s'], max_gpt_calls_per_min=p['max_gpt_calls_per_min'],
            repeat_window_s=p['repeat_window_s'], use_image=p['use_image'] and p['camera'] != 'none',
            image_max_side=p['image_max_side'], jpeg_quality=p['jpeg_quality'],
            interrupt_on_fault=p['interrupt_on_fault'], fault_tts_deadline_s=p['fault_tts_timeout_s'] + 1.0,
            gpt_state_deadline_s=p['gpt_state_deadline_s'], tts_deadline_s=p['tts_deadline_s'],
            gpt_narrate_deadline_s=p['gpt_narrate_deadline_s'], gpt_ask_deadline_s=p['gpt_ask_deadline_s'],
            wake_mode=p['wake_mode'], wake_ack=p['wake_ack'], gpt_voice_deadline_s=p['gpt_voice_deadline_s'],
            notice=p['notice'], reactions=load_reactions(p['reactions'], log),
            chatter_period_s=p['chatter_period_s'], chatter_jitter=p['chatter_jitter'],
            chatter_quiet_s=p['chatter_quiet_s'])
        if p['state_mode'] not in ('gpt', 'template'):
            raise ValueError("state_mode must be 'gpt' or 'template'")

        backend = None
        from .gpt import openai_auth_available
        if openai_auth_available():
            try:
                from .gpt import OpenAIBackend
                backend = OpenAIBackend(
                    model=p['model'], tts_model=p['tts_model'], voice=p['tts_voice'],
                    tts_instructions=p['tts_instructions'], image_detail=p['image_detail'],
                    max_output_tokens=p['max_output_tokens'], reasoning_effort=p['reasoning_effort'],
                    tts_speed=p['tts_speed'], sample_rate=p['tts_sample_rate'],
                    timeout_s=p['request_timeout_s'], urgent_timeout_s=p['fault_tts_timeout_s'],
                    base_url=p['api_base_url'], stt_model=p['stt_model'])
            except Exception as error:  # e.g. openai package missing
                log.error(f'OpenAI backend unavailable ({type(error).__name__}: {error}); templates only')
        else:
            log.warning('OPENAI_API_KEY is not set: no GPT and no OpenAI speech, templates only')
        fallback = split_command(p['fallback_tts_command'])
        player = split_command(p['player_command'])

        self._utterance_pub = self.create_publisher(String, '/voice/utterance', 10)
        self._heard_pub = self.create_publisher(String, '/voice/heard', 10)
        self._speaking_pub = self.create_publisher(Bool, '/voice/speaking', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, '/voice/diagnostics', 10)
        self._frames = None
        self._setup_camera(p)

        # Streaming plays on the same speaker as player_command (device options kept); none without a speaker.
        stream = []
        if p['tts_stream'] and backend and player:
            stream = split_command(p['stream_player_command']) or stream_command_for(player) or []
            if not stream:
                log.info(f'no raw-PCM variant of {player[0]!r}: speaking whole WAVs (set stream_player_command)')
        self.narrator = Narrator(
            config, tracker, backend=backend, player=CommandPlayer(player) if player else None,
            stream_player=StreamPlayer(stream) if stream else None,
            frame_provider=self._frames.pick if config.use_image and self._frames else None,
            fallback_tts=CommandTts(fallback) if fallback else None,
            on_text=self._publish_text, on_speaking=self._publish_speaking, logger=log,
            speech_cache=make_cache(backend, p['speech_cache_dir']),
            quick_answer=self._menu(p))
        self.listener = None
        if p['listen']:
            self.listener = make_listener(
                self.narrator, backend, log, mic_command=p['mic_command'], wake_words=p['wake_words'],
                language=p['stt_language'], stt_prompt=p['stt_prompt'], follow_up_s=p['follow_up_s'],
                tail_s=p['listen_tail_s'], max_stt_per_min=p['max_stt_per_min'], tolerance=p['wake_tolerance'],
                min_rms=p['vad_min_rms'], loud_ratio=p['vad_loud_ratio'],
                on_heard=lambda question: self._heard_pub.publish(String(data=question)),
                words=self._word_reactions(p['word_reactions'], log))
            if self.listener is not None:
                self.listener.start()

        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)  # matches planning_master
        self.create_subscription(String, p['planning_state_topic'],
                                 lambda m: self.narrator.planning_state(m.data), latched)
        self.create_subscription(DiagnosticArray, p['planning_diagnostics_topic'], self._on_diagnostics, 10)
        try:
            from craft_interfaces.msg import ControlStatus
            check_for_type_support(ControlStatus)
            self.create_subscription(ControlStatus, p['control_status_topic'], lambda m: self.narrator.control_status(
                m.ready, m.fault, m.held, m.stopped, m.reason), 10)
        except (ImportError, UnsupportedTypeSupport):
            log.warning('craft_interfaces type support unavailable: control status is left out of the context')
        events = parse_list(p['event_topics'])
        self.narrator.tracker.event_topics = tuple(events)
        for topic in dict.fromkeys(parse_list(p['context_topics']) + events):  # volatile: latched and plain publishers
            self.create_subscription(String, topic, lambda m, t=topic: self.narrator.context(t, m.data), 10)
        for topic in parse_list(p['flag_topics']):
            self.create_subscription(Bool, topic, lambda m, t=topic: self.narrator.context(
                t, 'true' if m.data else 'false'), 10)
        self._subscribe_perception(p)
        if p['variety'] or p['obstacles_topic']:
            self._subscribe_goal(p)   # the obstacles are in the map frame: the robot's pose places them
        self._subscribe_signals(p, latched)
        self.create_subscription(String, '/voice/ask', lambda m: self.narrator.ask(m.data), 10)
        self.create_subscription(String, '/voice/say', lambda m: self.narrator.say(m.data), 10)
        self.create_subscription(String, '/voice/command', self._on_command, 10)
        self.create_timer(0.1, self.narrator.tick)
        self.create_timer(1.0, self._publish_diagnostics)
        log.info(f"voice ready: camera={p['camera']} gpt={'on' if backend else 'off'} model={p['model']} "
                 f"tts={p['tts_model']}/{p['tts_voice']} state_mode={p['state_mode']} style={p['style']} "
                 f"narrate every {p['narrate_period_s']}s player={' '.join(player) or 'none'} "
                 f"stream={' '.join(stream) or 'off'} "
                 f"microphone={'on (' + p['wake_words'] + ')' if self.listener else 'off'}")
        self._clock_unset = clock_unset()
        if self._clock_unset:
            log.warning('the system clock is not set yet (no clock battery): fixed sentences are spoken, '
                        'the microphone and GPT wait until it is set')

    def _menu(self, p):
        if not p['menu_file'].strip():
            return None
        from .menu import Menu
        menu = Menu(p['menu_file'], style=p['style'], items=p['menu_items'], logger=self.get_logger())
        self.get_logger().info(f"menu: {len(menu.days())} days from {menu.path}")
        return menu.answer

    # ---- what perception sees ---------------------------------------------------------------
    def _subscribe_perception(self, p):
        log = self.get_logger()
        if p['odom_topic']:
            try:
                from nav_msgs.msg import Odometry
                self.create_subscription(Odometry, p['odom_topic'], self._on_odometry, qos_profile_sensor_data)
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
            if p['objects_topic']:
                check_for_type_support(DetectionArray)
            if p['pucks_topic']:
                check_for_type_support(PuckObservationArray)
        except (ImportError, UnsupportedTypeSupport):
            log.warning('craft_perception type support unavailable: YOLO objects and pucks are left out')
            return
        if p['objects_topic']:
            def on_objects(m, topic=p['objects_topic']):
                self._input_frame(m.header, topic)
                self.narrator.objects([d.label for d in m.detections])
            self.create_subscription(DetectionArray, p['objects_topic'], on_objects, qos_profile_sensor_data)
        if p['pucks_topic']:
            def on_pucks(m, topic=p['pucks_topic']):
                self._input_frame(m.header, topic)
                # Without a depth axis the centre is (0,0,0) and 'standing' is meaningless: report unknown.
                self.narrator.pucks([(o.standing, math.sqrt(o.center.x ** 2 + o.center.y ** 2 + o.center.z ** 2))
                                     if o.axis_valid else (None, None) for o in m.observations])
            self.create_subscription(PuckObservationArray, p['pucks_topic'], on_pucks, qos_profile_sensor_data)

    def _word_reactions(self, spec: str, log) -> dict:
        """'젯슨|제슨|jetson=~/a.wav;단어2=~/b.wav' -> {'젯슨': ('젯슨', '제슨', 'jetson')}, 녹음은 reactions 의
        'word.젯슨' 으로. 읽을 수 없는 녹음의 단어는 빼고 경고한다."""
        words = {}
        for entry in (spec or '').split(';'):
            names, _, path = entry.partition('=')
            aliases = tuple(a.strip() for a in names.split('|') if a.strip())
            if not aliases or not path.strip():
                continue
            loaded = load_reactions(f'word.{aliases[0]}={path.strip()}', log)
            if loaded:
                self.narrator.config.reactions.update(loaded)
                words[aliases[0]] = aliases
                log.info(f'word {aliases[0]!r} ({", ".join(aliases)}) -> {path.strip()}')
        return words

    def _subscribe_signals(self, p, latched):
        """Control's task, planning's command, the barcode, the LiDAR obstacles, the box observation."""
        log = self.get_logger()
        try:
            from craft_interfaces.msg import BoxObservation, TaskCommand, TaskResult
            for kind in (BoxObservation, TaskCommand, TaskResult):
                check_for_type_support(kind)
        except (ImportError, UnsupportedTypeSupport):
            log.warning('craft_interfaces type support unavailable: task result, task command and box are left out')
        else:
            if p['task_result_topic']:
                last = {}

                def on_result(m):  # every control cycle: only a change, or one a second, goes further
                    key, now = (m.task_id, m.status, bytes(m.controllers), m.reason), time.monotonic()
                    if key != last.get('key') or now - last.get('at', 0.0) >= 1.0:
                        last.update(key=key, at=now)
                        self.narrator.task_result(m.task_id, m.status, list(m.controllers), m.reason)
                self.create_subscription(TaskResult, p['task_result_topic'], on_result, 10)
            if p['task_command_topic']:
                self.create_subscription(TaskCommand, p['task_command_topic'], lambda m: self.narrator.task_command(
                    m.task_id, m.operation, m.kind, m.duration_s, m.clock_slot, m.close), 10)
            if p['box_topic']:
                self.create_subscription(BoxObservation, p['box_topic'], lambda m: self.narrator.box(
                    m.valid, m.source, m.reason, m.gap_m, m.lateral_m, m.yaw_rad), 10)
        if p['barcode_topic']:
            started = time.monotonic()  # a barcode read before the voice started is known, not announced
            self.create_subscription(String, p['barcode_topic'], lambda m: self.narrator.barcode(
                m.data, quiet=time.monotonic() - started < 3.0), latched)
        if p['traffic_light_topic']:
            try:
                from craft_perception.msg import TrafficLight
                check_for_type_support(TrafficLight)
            except (ImportError, UnsupportedTypeSupport):
                log.warning('craft_perception TrafficLight unavailable: the traffic light is left out')
            else:
                light_started = time.monotonic()  # a count finished before the voice started is known, not said
                self.create_subscription(TrafficLight, p['traffic_light_topic'], lambda m: self.narrator.traffic_light(
                    m.red, m.yellow, m.green, len(m.sequence), m.complete, m.valid, m.discarded_cycles,
                    quiet=time.monotonic() - light_started < 3.0), latched)
        if p['obstacles_topic']:
            try:
                from geometry_msgs.msg import PoseArray
            except ImportError:
                log.warning('geometry_msgs not found: LiDAR obstacles are left out')
                return
            seen = {'at': 0.0}

            def on_obstacles(m):  # 10 Hz scans: three a second are enough for "what is near"
                now = time.monotonic()
                if now - seen['at'] < 0.3:
                    return
                seen['at'] = now
                self.narrator.obstacles([(q.position.x, q.position.y) for q in m.poses],
                                        in_map=m.header.frame_id.lstrip('/') == 'map')
            self.create_subscription(PoseArray, p['obstacles_topic'], on_obstacles, qos_profile_sensor_data)

    def _subscribe_goal(self, p):
        """Where the goal is and where the robot is, for 'N미터 앞으로 갈게' and 'N미터쯤 남았어'."""
        try:
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Odometry
        except ImportError:
            self.get_logger().warning('geometry_msgs / nav_msgs not found: drives are said without distance')
            return
        if p['goal_topic']:
            self.create_subscription(PoseStamped, p['goal_topic'], lambda m: self.narrator.goal(
                m.pose.position.x, m.pose.position.y), 10)
        if p['pose_topic']:
            def on_pose(m):
                q = m.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.narrator.pose(m.pose.pose.position.x, m.pose.pose.position.y, yaw)
            self.create_subscription(Odometry, p['pose_topic'], on_pose, qos_profile_sensor_data)

    def _on_odometry(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if 0 <= age <= .6:
            self.narrator.motion(msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _input_frame(self, header, topic: str):
        """planning_master acts on these same messages; their stamp is the camera frame they came from."""
        if self._frames is not None:
            self._frames.on_input(stamp_ns(getattr(header, 'stamp', None)), topic)

    # ---- camera -----------------------------------------------------------------------------
    def _setup_camera(self, p):
        camera = p['camera']
        if camera == 'none':
            return
        if camera not in SHM_CAMERAS and camera != 'topic':
            raise ValueError("camera must be d435, c310, topic or none")
        if camera == 'topic' and p['image_transport'] not in ('raw', 'compressed'):
            raise ValueError("image_transport must be 'raw' or 'compressed'")
        reader = None
        if camera in SHM_CAMERAS:
            name, depth, _ = SHM_CAMERAS[camera]
            reader = ShmFrameReader(name, depth)
        self._frames = CameraFrames(reader, mode=p['image_source'], max_age_s=p['frame_max_age_s'],
                                    min_interval_s=p['input_frame_interval_s'])
        if camera in SHM_CAMERAS:
            self.create_subscription(UInt64MultiArray, SHM_CAMERAS[camera][2],
                                     lambda m: self._frames.on_meta(m.data), qos_profile_sensor_data)
        else:
            if p['image_transport'] == 'compressed':
                from sensor_msgs.msg import CompressedImage as ImageType
            else:
                from sensor_msgs.msg import Image as ImageType
            self.create_subscription(ImageType, p['image_topic'], lambda m: self._frames.on_image(
                m, stamp_ns(getattr(getattr(m, 'header', None), 'stamp', None))), qos_profile_sensor_data)

    # ---- callbacks / outputs ----------------------------------------------------------------
    def _on_diagnostics(self, msg: DiagnosticArray):
        for status in msg.status:
            self.narrator.diagnostics(status.name, int.from_bytes(status.level, 'little')
                                      if isinstance(status.level, bytes) else int(status.level),
                                      status.message, {kv.key: kv.value for kv in status.values})

    def _on_command(self, msg: String):
        command = msg.data.strip().lower()
        if command in ('listen', 'deaf'):  # microphone on / off while running
            if self.listener is not None:
                self.listener.enabled = command == 'listen'
            result = command if self.listener is not None else 'no microphone (listen:=false)'
        elif command == 'wake':            # as if the name was called
            self.narrator.wake('')
            result = command
        else:
            result = self.narrator.command(msg.data)
        self.get_logger().info(f'/voice/command: {result}')

    def _publish_text(self, kind: str, text: str):
        self._utterance_pub.publish(String(data=f'{kind}: {text}'))

    def _publish_speaking(self, speaking: bool):
        self._speaking_pub.publish(Bool(data=speaking))

    def _publish_diagnostics(self):
        if self._clock_unset and not clock_unset():
            self._clock_unset = False
            self.get_logger().info('the system clock is set: listening and GPT are on')
        stats = self.narrator.status()
        stats['clock_set'] = not self._clock_unset
        status = DiagnosticStatus(name='voice/narrator', hardware_id='voice_node',
                                  message='muted' if stats['muted'] else 'ok')
        status.level = DiagnosticStatus.OK
        if self.listener is not None:
            stats.update({f'mic_{key}': value for key, value in self.listener.stats.items()},
                         mic_enabled=self.listener.enabled)
        status.values = [KeyValue(key=key, value=str(value)) for key, value in sorted(stats.items())]
        if self._frames is not None:
            latest, newest = self._frames.ages()
            status.values.append(KeyValue(key='frame_age_s', value=f'{latest:.2f}'))
            status.values.append(KeyValue(key='input_frame_age_s', value=f'{newest:.2f}'))
            status.values += [KeyValue(key=key, value=str(value)) for key, value in sorted(self._frames.stats.items())]
        msg = DiagnosticArray(status=[status])
        msg.header.stamp = self.get_clock().now().to_msg()
        self._diagnostics_pub.publish(msg)

    def destroy_node(self):
        if self.listener is not None:
            self.listener.close()
        self.narrator.close()
        if self._frames is not None:
            self._frames.close()
        super().destroy_node()


def main(args=None):
    from rclpy.executors import ExternalShutdownException

    from .envfile import load_env_file
    load_env_file()  # OPENAI_API_KEY from env/openai.env or ~/.config/craft_voice/openai.env, if not already set
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C, or SIGTERM from systemd
        pass
    except Exception:
        if rclpy.ok():  # after SIGTERM the wait set can fail before ExternalShutdownException is raised
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
