#!/usr/bin/env python3
"""Recover from mission/planning fault, relocalize via scan matching, detour around obstacles, and return to home with scan_fit >= 90%."""
import math
import re
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

MISSION_YAML = Path("/home/craft/craft/src/craft_planning/config/mission.yaml")
SLAM_MAP_YAML = "/home/craft/maps/brief_arena1cm.yaml"
GLOBAL_LOCALIZE_BIN = "/home/craft/craft/install/craft_bringup/lib/craft_bringup/global_localize"

BODY_CENTER_X, BODY_R = 0.06075, 0.27
FIT_TOL = 0.05
TARGET_FIT = 0.90

BOXES = {
    "S1": (1.476, 1.159),
    "S2": (3.616, 3.436),
    "S3": (3.069, 5.336),
    "S4": (0.864, 5.389),
    "S5": (2.272, 5.321),
    "S6": (3.611, 3.889),
    "L1": (3.576, 1.066),
    "L2": (2.349, 3.726),
    "L3": (3.754, 5.324),
}


def load_home_pose():
    default = (0.662, 1.081, -179.6)
    if not MISSION_YAML.exists():
        return default
    text = MISSION_YAML.read_text(encoding="utf-8", errors="ignore")
    m = re.search(r"^\s*home:\s*\[\s*([0-9.+-]+)\s*,\s*([0-9.+-]+)\s*,\s*([0-9.+-]+)\s*\]", text, re.M)
    if not m:
        return default
    return float(m.group(1)), float(m.group(2)), float(m.group(3))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    return math.remainder(a, 2.0 * math.pi)


def main():
    hx, hy, hyaw_deg = load_home_pose()
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if len(args) >= 2:
        hx, hy = float(args[0]), float(args[1])
    if len(args) >= 3:
        hyaw_deg = float(args[2])
    hyaw = math.radians(hyaw_deg)

    rclpy.init()
    node = rclpy.create_node("go_home_helper")

    state = {
        "planning": "",
        "mission": "",
        "odom": None,
        "scan": None,
        "map_dist": None,
        "loc_global": "",
        "loc_global_at": 0.0,
        "waiting_for": "",
        "path_revision": "",
    }

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/planning/state", lambda m: state.__setitem__("planning", m.data), 10)
    node.create_subscription(String, "/mission/state", lambda m: state.__setitem__("mission", m.data), latched)
    node.create_subscription(Odometry, "/perception/state/odom", lambda m: state.__setitem__("odom", m), 10)
    node.create_subscription(LaserScan, "/lidar/merged", lambda m: state.__setitem__("scan", m), qos_profile_sensor_data)

    def on_diag(msg):
        for s in msg.status:
            if s.name == "planning/state_machine":
                kv = {v.key: v.value for v in s.values}
                state["waiting_for"] = kv.get("waiting_for", "")
                state["path_revision"] = kv.get("path_revision", "")

    node.create_subscription(DiagnosticArray, "/planning/diagnostics", on_diag, 10)

    def on_loc_global(m):
        state["loc_global"] = m.data
        state["loc_global_at"] = time.monotonic()

    node.create_subscription(String, "/perception/localization/global", on_loc_global, latched)
    node.create_subscription(String, "/home/localization/global", on_loc_global, latched)
    loc_ok_pub = node.create_publisher(String, "/perception/localization/global", latched)

    def on_map(m):
        if state["map_dist"] is not None:
            return
        w, h = m.info.width, m.info.height
        grid = np.array(m.data, dtype=np.int16).reshape(h, w)
        occupied = grid >= 65
        dist = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 3) * m.info.resolution
        state["map_dist"] = (dist, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, w, h)

    node.create_subscription(OccupancyGrid, "/planning/map", on_map, latched)
    node.create_subscription(OccupancyGrid, "/perception/lidar/map", on_map, latched)

    mission_pub = node.create_publisher(String, "/mission/command", 10)
    planning_pub = node.create_publisher(String, "/planning/command", 10)
    leave_pub = node.create_publisher(String, "/planning/leave_box", 10)
    goal_pub = node.create_publisher(PoseStamped, "/planning/target_base_pose", 10)
    teleop_pub = node.create_publisher(Twist, "/control/teleop/cmd_vel", 10)
    global_req_pub = node.create_publisher(Empty, "/perception/localization/global_request", 10)

    def spin_for(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.04)

    def send_str(pub, text):
        msg = String()
        msg.data = text
        pub.publish(msg)

    def stop_wheels():
        z = Twist()
        for _ in range(5):
            teleop_pub.publish(z)
            spin_for(0.03)

    def current_pose():
        o = state["odom"]
        if o is None:
            return None
        p = o.pose.pose
        return p.position.x, p.position.y, quat_to_yaw(p.orientation)

    def compute_scan_fit():
        scan = state["scan"]
        pose = current_pose()
        md = state["map_dist"]
        if scan is None or pose is None or md is None:
            return None
        dist, res, ox, oy, w, h = md
        r = np.asarray(scan.ranges, float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > 0.3) & (r < 6.0)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        keep = np.hypot(x - BODY_CENTER_X, y) > BODY_R
        x, y = x[keep], y[keep]
        if len(x) < 30:
            return None
        c, s = math.cos(pose[2]), math.sin(pose[2])
        mx, my = pose[0] + c * x - s * y, pose[1] + s * x + c * y
        col, row = ((mx - ox) / res).astype(int), ((my - oy) / res).astype(int)
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        d = np.full(len(mx), np.inf)
        d[inside] = dist[row[inside], col[inside]]
        return float(np.mean(d < FIT_TOL))

    def relocalize(near_home=False):
        """Run C++ global_localize scan matching."""
        stop_wheels()
        send_str(planning_pub, "STOP")
        spin_for(0.3)

        if near_home:
            print("[home] 시작 구역 스캔매칭(global_localize) 요청 중...")
            t_req = time.monotonic()
            global_req_pub.publish(Empty())
            while time.monotonic() - t_req < 6.0:
                spin_for(0.1)
                if state["loc_global_at"] > t_req and state["loc_global"].startswith("ok "):
                    fit = compute_scan_fit() or 0.0
                    print(f"[home] 스캔매칭 완료: {state['loc_global']} (scan_fit={fit*100:.1f}%)")
                    if fit >= TARGET_FIT:
                        return True
                    break

        print("[home] 전역 스캔매칭(C++ global_localize) 실행 중...")
        cmd = [
            GLOBAL_LOCALIZE_BIN,
            "--map", SLAM_MAP_YAML,
            "--ros-args", "-r", "__node:=home_global_localize", "-r", "/perception/localization/global:=/home/localization/global",
        ]
        t_start = time.monotonic()
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            while time.monotonic() - t_start < 12.0:
                spin_for(0.1)
                if state["loc_global_at"] > t_start and state["loc_global"].startswith("ok "):
                    spin_for(0.4)
                    fit = compute_scan_fit() or 0.0
                    cp = current_pose()
                    print(f"[home] 전역 스캔매칭 완료: {state['loc_global']} | 현재 odom=({cp[0]:.3f}, {cp[1]:.3f}, {math.degrees(cp[2]):.1f} deg), scan_fit={fit*100:.1f}%")
                    return fit >= TARGET_FIT
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        fit = compute_scan_fit() or 0.0
        return fit >= TARGET_FIT

    def ensure_await_request(timeout=10.0):
        t0 = time.monotonic()
        last_cmd = 0.0
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            ps = state["planning"]
            now = time.monotonic()
            if ps == "Run.AwaitRequest":
                return True
            if now - last_cmd >= 0.4:
                if ps == "Fault":
                    send_str(planning_pub, "RESET")
                elif ps == "Ready":
                    send_str(planning_pub, "START")
                elif ps.startswith("Run.") and ps != "Run.AwaitRequest":
                    send_str(planning_pub, "STOP")
                last_cmd = now
        return False

    def wait_drive_done(timeout=180.0, watch_obstacles=True):
        t0 = time.monotonic()
        busy = False
        last_wait_msg = ""
        last_rev = state["path_revision"]
        blocked_at = None
        last_move_pose = current_pose()
        last_move_at = time.monotonic()

        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            ps = state["planning"]
            wf = state["waiting_for"]
            rev = state["path_revision"]
            now = time.monotonic()

            cp = current_pose()
            if cp and last_move_pose:
                if math.hypot(cp[0] - last_move_pose[0], cp[1] - last_move_pose[1]) >= 0.02 or abs(wrap_pi(cp[2] - last_move_pose[2])) >= math.radians(3.0):
                    last_move_pose = cp
                    last_move_at = now

            if watch_obstacles:
                if wf != last_wait_msg:
                    if wf:
                        print(f"[home] 주행 상태: {wf}")
                    last_wait_msg = wf
                if rev and rev != last_rev:
                    if last_rev:
                        print(f"[home] 장애물 우회 경로 갱신 완료 (revision={rev}) -> 우회 주행 진행")
                    last_rev = rev
                    blocked_at = None

                # Detect if obstacle replanning is stuck because robot stopped inside obstacle padding
                if any(k in wf for k in ("route unavailable", "Start is in an obstacle", "path blocked", "path still blocked", "local path")):
                    if blocked_at is None:
                        blocked_at = now
                    elif ("Start is in an obstacle" in wf or "route unavailable" in wf) or (now - blocked_at > 4.5 and now - last_move_at > 4.5):
                        return False, f"ObstacleStuck: {wf}"
                else:
                    blocked_at = None

            if ps == "Fault":
                return False, "Fault"
            if ps == "Run.Complete":
                return True, "Complete"
            if ps.startswith("Run.") and ps != "Run.AwaitRequest":
                busy = True
            elif ps == "Run.AwaitRequest" and busy:
                return True, "Complete"
        return False, "Timeout"

    def align_heading(target_yaw):
        send_str(planning_pub, "STOP")
        spin_for(0.35)
        if state["planning"] == "Fault":
            send_str(planning_pub, "RESET")
            spin_for(0.25)

        print(f"[home] 시작 방향({math.degrees(target_yaw):.1f} deg) 정렬 중...")
        t_rot = time.monotonic()
        settled_since = None
        tol_rad = math.radians(1.0)
        while time.monotonic() - t_rot < 25.0:
            rclpy.spin_once(node, timeout_sec=0.04)
            cp = current_pose()
            if cp is None:
                continue
            err = wrap_pi(target_yaw - cp[2])
            if abs(err) <= tol_rad:
                stop_wheels()
                if settled_since is None:
                    settled_since = time.monotonic()
                elif time.monotonic() - settled_since >= 0.25:
                    return True
            else:
                settled_since = None
                cmd = Twist()
                w = max(0.10, min(0.45, 1.2 * abs(err)))
                cmd.angular.z = math.copysign(w, err)
                teleop_pub.publish(cmd)
        stop_wheels()
        return False

    try:
        print(f"[home] 목표 시작 위치: ({hx:.3f}, {hy:.3f}), 방향: {hyaw_deg:.1f} deg (장애물 우회 + 스캔매칭 scan_fit >= 90%)")
        spin_for(0.8)

        if state["odom"] is None or state["scan"] is None or state["map_dist"] is None:
            print("[home] 오도메트리/라이다/맵 토픽 수신 대기 중...")
            spin_for(2.0)
        if state["odom"] is None or state["scan"] is None or state["map_dist"] is None:
            print("[home] 필수 토픽(/perception/state/odom, /lidar/merged, /planning/map)이 수신되지 않습니다.")
            return 1

        # 1. Stop and reset mission_master
        send_str(mission_pub, "STOP")
        spin_for(0.2)
        send_str(mission_pub, "RESET")
        spin_for(0.2)

        for round_idx in range(1, 6):
            cp = current_pose()
            fit = compute_scan_fit() or 0.0
            print(f"[home] [점검 {round_idx}] 현재 위치: ({cp[0]:.3f}, {cp[1]:.3f}, {math.degrees(cp[2]):.1f} deg), scan_fit={fit*100:.1f}%")

            if fit < TARGET_FIT:
                print(f"[home] scan_fit({fit*100:.1f}%) < 90% -> 전역 스캔매칭 보정을 수행합니다.")
                relocalize(near_home=False)
                cp = current_pose()
                fit = compute_scan_fit() or 0.0

            dist_to_home = math.hypot(cp[0] - hx, cp[1] - hy)
            yaw_err_deg = abs(math.degrees(wrap_pi(hyaw - cp[2])))

            if dist_to_home <= 0.035 and yaw_err_deg <= 1.5 and fit >= TARGET_FIT:
                for _ in range(5):
                    send_str(loc_ok_pub, f"ok {cp[0]:.3f} {cp[1]:.3f} {math.degrees(cp[2]):.1f} {fit:.2f}")
                    spin_for(0.1)
                send_str(mission_pub, "RESET")
                print(f"[home] 복귀 및 검증 완료! 위치: ({cp[0]:.3f}, {cp[1]:.3f}), 방향: {math.degrees(cp[2]):.1f} deg, scan_fit: {fit*100:.1f}%")
                return 0

            if dist_to_home > 0.03:
                if not ensure_await_request(timeout=10.0):
                    print(f"[home] planning_master 준비 실패 (현재: {state['planning']})")
                    return 1

                nearest_box, nearest_dist = None, 1e9
                for bname, (bx, by) in BOXES.items():
                    d = math.hypot(cp[0] - bx, cp[1] - by)
                    if d < nearest_dist:
                        nearest_box, nearest_dist = bname, d

                if nearest_dist < 0.48:
                    print(f"[home] 박스 {nearest_box} 근접({nearest_dist:.2f} m) -> 후진 탈출(leave_box) 실행")
                    send_str(leave_pub, nearest_box)
                    wait_drive_done(timeout=45.0, watch_obstacles=False)
                    ensure_await_request(timeout=10.0)

                for attempt in range(4):
                    if not ensure_await_request(timeout=10.0):
                        break
                    goal = PoseStamped()
                    goal.header.frame_id = "map"
                    goal.header.stamp = node.get_clock().now().to_msg()
                    goal.pose.position.x = hx
                    goal.pose.position.y = hy
                    goal.pose.orientation.z = math.sin(hyaw / 2.0)
                    goal.pose.orientation.w = math.cos(hyaw / 2.0)
                    print(f"[home] 시작 위치 ({hx:.3f}, {hy:.3f})로 주행 요청 (장애물 감지 시 자동 우회)...")
                    goal_pub.publish(goal)
                    ok, reason = wait_drive_done(timeout=180.0, watch_obstacles=True)
                    if ok:
                        break

                    print(f"[home] 장애물/경로 막힘 감지({reason}) -> 공간 확보 및 우회 재시도...")
                    fit_now = compute_scan_fit() or 0.0
                    if fit_now < TARGET_FIT:
                        print(f"[home] 주행 중 위치 틀어짐(scan_fit={fit_now*100:.1f}%) -> 스캔매칭 재보정")
                        relocalize(near_home=False)
                    if ensure_await_request(timeout=10.0):
                        print("[home] 장애물 패딩 밖으로 직선 후진(leave_box) 후 우회 경로를 다시 계획합니다.")
                        send_str(leave_pub, nearest_box or "obstacle")
                        wait_drive_done(timeout=30.0, watch_obstacles=False)

            align_heading(hyaw)
            spin_for(0.5)

            relocalize(near_home=True)
            spin_for(0.4)
            fit = compute_scan_fit() or 0.0
            cp = current_pose()
            dist_to_home = math.hypot(cp[0] - hx, cp[1] - hy)
            yaw_err_deg = abs(math.degrees(wrap_pi(hyaw - cp[2])))
            if fit >= TARGET_FIT and dist_to_home <= 0.035 and yaw_err_deg <= 1.5:
                for _ in range(5):
                    send_str(loc_ok_pub, f"ok {cp[0]:.3f} {cp[1]:.3f} {math.degrees(cp[2]):.1f} {fit:.2f}")
                    spin_for(0.1)
                send_str(mission_pub, "RESET")
                print(f"[home] 복귀 및 검증 완료! 위치: ({cp[0]:.3f}, {cp[1]:.3f}), 방향: {math.degrees(cp[2]):.1f} deg, scan_fit: {fit*100:.1f}%")
                return 0
            print(f"[home] 도착 후 재검증: 거리오차={dist_to_home*1000:.1f} mm, 각도오차={yaw_err_deg:.1f} deg, scan_fit={fit*100:.1f}% -> 재보정 및 재주행 진행")
            relocalize(near_home=False)

        print("[home] 최대 재시도 횟수(5회)를 초과했습니다.")
        return 1

    except KeyboardInterrupt:
        print("\n[home] 사용자 중단(Ctrl+C) -> 정지합니다.")
        send_str(planning_pub, "STOP")
        stop_wheels()
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
