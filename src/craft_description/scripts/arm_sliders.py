#!/usr/bin/env python3
"""고른 관절만 슬라이더로 움직이는 joint_states 발행 노드.

joint_state_publisher_gui는 움직이는 관절을 전부 띄우므로, LM 가이드와 팔 관절만
슬라이더로 보이게 하고 나머지 움직이는 관절은 0으로 고정해 함께 발행한다.

파라미터
  robot_description (str): URDF 문자열
  joints (str[]): 슬라이더로 띄울 관절 이름. 비우면 움직이는 관절 전부
  rate (float): 발행 주기 [Hz]
"""
import math
import threading
import tkinter as tk
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

DEFAULT_JOINTS = ['lm_slide', 'shoulder', 'elbow', 'wrist_pitch', 'tool_roll']


def parse_movable_joints(urdf):
    """URDF에서 움직이는 관절을 {이름: (type, lower, upper)}로 순서대로 반환."""
    joints = {}
    for j in ET.fromstring(urdf).iter('joint'):
        jtype = j.get('type')
        if jtype in ('fixed', 'floating', 'planar') or j.find('mimic') is not None:
            continue
        limit = j.find('limit')
        if jtype == 'continuous' or limit is None:
            lower, upper = -math.pi, math.pi
        else:
            lower = float(limit.get('lower', 0.0))
            upper = float(limit.get('upper', 0.0))
        joints[j.get('name')] = (jtype, lower, upper)
    return joints


class ArmSliders(Node):
    def __init__(self):
        super().__init__('arm_sliders')
        urdf = self.declare_parameter('robot_description', '').value
        wanted = self.declare_parameter('joints', DEFAULT_JOINTS).value
        rate = self.declare_parameter('rate', 30.0).value
        if not urdf:
            raise RuntimeError('robot_description 파라미터가 비어 있음')

        self.movable = parse_movable_joints(urdf)
        wanted = [n for n in wanted if n] or list(self.movable)
        missing = [n for n in wanted if n not in self.movable]
        if missing:
            self.get_logger().warning(f'URDF에 없는(또는 고정된) 관절이라 건너뜀: {missing}')
        self.slider_joints = [n for n in wanted if n in self.movable]

        # 0이 범위 밖이면 범위 안의 가장 가까운 값에서 시작
        self.positions = {n: min(max(0.0, lo), hi) for n, (_, lo, hi) in self.movable.items()}
        self.lock = threading.Lock()
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0 / rate, self.publish)

    def set_position(self, name, value):
        with self.lock:
            self.positions[name] = value

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.name = list(self.positions)
            msg.position = list(self.positions.values())
        self.pub.publish(msg)


def build_gui(node):
    root = tk.Tk()
    root.title('Joint sliders')
    root.geometry('+60+60')
    scales = {}

    for row, name in enumerate(node.slider_joints):
        jtype, lo, hi = node.movable[name]
        prismatic = jtype == 'prismatic'
        # 직동은 mm, 회전은 deg로 표시
        k = 1000.0 if prismatic else math.degrees(1.0)
        unit = 'mm' if prismatic else 'deg'
        tk.Label(root, text=f'{name} [{unit}]', anchor='w').grid(
            row=row, column=0, sticky='w', padx=8)
        scale = tk.Scale(root, from_=lo * k, to=hi * k, orient=tk.HORIZONTAL,
                         length=420, resolution=0.1 if prismatic else 0.5,
                         command=lambda v, n=name, k=k: node.set_position(n, float(v) / k))
        scale.set(node.positions[name] * k)
        scale.grid(row=row, column=1, padx=8, pady=2)
        scales[name] = (scale, k)

    def center():
        for name, (scale, k) in scales.items():
            _, lo, hi = node.movable[name]
            scale.set(min(max(0.0, lo), hi) * k)

    tk.Button(root, text='Center', command=center).grid(
        row=len(scales), column=0, columnspan=2, pady=6)
    return root


def main():
    rclpy.init()
    node = ArmSliders()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    root = build_gui(node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
