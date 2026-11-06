#!/usr/bin/env python3
"""Throwaway demo: teach arm poses. Streams the lm_arm board feedback live (control_master must be off) and saves
the pose on a key press to teach.yaml, raw motor degrees and the MANIPULATOR target execute.py uses.

Two ways to put the arm somewhere:
  by hand: r = torque OFF (SUPPORT THE ARM, it drops; positions keep tracking), h = torque ON where it is.
  jog (torque on, the motors move themselves, so the pose has the same sag as a real run; never push a holding
  arm by hand, it overloads): j = jog on/off (starts from where the arm is), 1..6 = axis (LM, shoulder, elbow,
  wrist_pitch, tool_roll, gripper), + or = / - = one step up / down, [ / ] = step 0.1 / 0.5 / 1 / 5 motor degrees
  (the LM also 20 and 90: 2.2 / 9.9 mm).
  While jogging the target is sent 20 times a second; the motor limits (control.yaml) clamp it.
Enter = save the pose under the next name, u = drop the last saved pose, q = quit (the board keeps holding the last
target; torque stays as it is).
Usage:  teacharm [name ...]     (= python3 teach.py; names in order, default p1, p2, ...)
Delete with: rm -rf ~/craft/demo
"""
import math
import select
import shutil
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import serial
import yaml

HERE = Path(__file__).resolve().parent
CONTROL_YAML = HERE.parent.parent / 'src/craft_control/config/control.yaml'
PORT = '/dev/opencr_arm_lm'
NAMES = ['LM', 'shoulder', 'elbow', 'wrist_pitch', 'tool_roll', 'gripper']
FIELDS = 20  # LM..gripper, state, six currents [A], imu_ok, gx, gy, gz, ax, ay, az
OUT = HERE / 'teach.yaml'
STEPS = [0.1, 0.5, 1.0, 5.0]  # jog step [motor deg]
LM_STEPS = STEPS + [20.0, 90.0]  # the LM also: 20 deg = 2.2 mm, 90 deg = 9.9 mm
SEND_HZ = 20  # the firmware holds (state 2) when no target comes for 250 ms


def raw_to_target(raw_deg):
    """Raw motor degrees LM, shoulder, elbow, wrist_pitch, tool_roll, gripper -> MANIPULATOR target: URDF slide [m],
    shoulder, elbow, wrist_pitch, tool_roll [rad], gripper motor angle [rad] (control.yaml offsets/directions)."""
    c = yaml.safe_load(open(CONTROL_YAML))['control_master']['ros__parameters']
    lm, *arm, grip = [math.radians(v) for v in raw_deg]
    return ([(lm - c['lm_motor_offset']) * c['lm_motor_direction'] * c['lm_meters_per_rad']]
            + [(m - o) / d for m, o, d in zip(arm, c['arm_joint_offsets'], c['arm_joint_directions'])] + [grip])


def steps_of(axis):
    """Jog steps [motor deg] of an axis: the LM (0) up to 90 deg (9.9 mm), the arm joints up to 5 deg."""
    return LM_STEPS if axis == 0 else STEPS


def motor_limits():
    """[(min, max)] motor rad per axis LM..gripper, control.yaml."""
    c = yaml.safe_load(open(CONTROL_YAML))['control_master']['ros__parameters']
    return ([(c['lm_min_angle'], c['lm_max_angle'])] + list(zip(c['arm_min_angle'], c['arm_max_angle']))
            + [(c['gripper_min_angle'], c['gripper_max_angle'])])


def main():
    names = sys.argv[1:]
    try:
        s = serial.Serial(PORT, 115200, timeout=0.1, exclusive=True)
    except serial.SerialException as e:
        print(f'{PORT}: {e} (control_master running? stop it first)'); return 1
    s.dtr = True
    latest, stop, lock = [None], threading.Event(), threading.Lock()
    jog = {'on': False, 'target': None, 'axis': 1, 'step': 1}
    limits = motor_limits()

    def write(data):
        with lock:
            s.write(data)

    def reader():
        buf = b''
        while not stop.is_set():
            buf += s.read(4096)
            *lines, buf = buf.split(b'\n')
            for line in lines:
                try:
                    v = [float(x) for x in line.decode(errors='replace').split(',')]
                except ValueError:
                    continue  # also the once-a-second diagnostic line "D,..."
                if len(v) == FIELDS:
                    latest[0] = v

    def sender():
        while not stop.is_set():
            t = jog['target']
            if jog['on'] and t is not None:
                write((','.join(f'{x:.6f}' for x in t) + '\n').encode())
            time.sleep(1 / SEND_HZ)

    threading.Thread(target=reader, daemon=True).start()
    threading.Thread(target=sender, daemon=True).start()
    saved = yaml.safe_load(open(OUT)) if OUT.exists() else None
    saved = saved or []
    print(f'{len(saved)} poses already in {OUT.name}. r torque OFF (hold the arm) | h torque ON | j jog on/off, '
          '1-6 axis, +/- step, [ ] step size | Enter save | u undo | q quit')
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            v = latest[0]
            nxt = names[0] if names else f'p{len(saved) + 1}'
            a = jog['axis']
            if v is None:
                line = 'no feedback yet'
            else:
                # The jog part first: a line wider than the terminal wraps and \r redraws only its last row.
                line = ''
                if jog['on']:
                    tgt, act = math.degrees(jog['target'][a]), math.degrees(v[a])
                    line = (f'JOG [{a + 1}] {NAMES[a]} {act:.1f} -> {tgt:.1f} (err {act - tgt:+.2f}) '
                            f'step {steps_of(a)[jog["step"]]:g}  {v[7 + a]:+.2f} A | ')
                else:
                    line = f'axis [{a + 1}] {NAMES[a]} step {steps_of(a)[jog["step"]]:g} (j = jog) | '
                line += ' '.join(f'{n[:5]} {math.degrees(x):.1f}' for n, x in zip(NAMES, v[:6]))
                line += f' | st {int(v[6])}{" OFF" if int(v[6]) == 2 and not jog["on"] else ""} next {nxt}'
            line = line[:max(20, shutil.get_terminal_size().columns - 1)]
            sys.stdout.write('\r\033[K' + line); sys.stdout.flush()
            if not select.select([sys.stdin], [], [], 0.2)[0]:
                continue
            k = sys.stdin.read(1)
            if k == 'q':
                break
            if k in 'rh':
                if k == 'r':
                    jog['on'] = False
                write(b'RELAX\n' if k == 'r' else b'HOLD\n')
                print(f'\n{"torque OFF: support the arm" if k == "r" else "torque ON, holding here"}')
            elif k == 'j' and v is not None:
                if jog['on']:
                    jog['on'] = False
                    print('\njog off: the board holds the last target')
                elif int(v[6]) == 2 and v[7:13] == [0.0] * 6:
                    print('\ntorque looks off (relaxed or fault): h first, then j')
                elif any(not lo <= x <= hi for x, (lo, hi) in zip(v[:6], limits)):
                    # The board drops a whole command when one axis is out of its limits: nothing would move.
                    out = [f'{n} {math.degrees(x):.1f} (limits {math.degrees(lo):.1f}..{math.degrees(hi):.1f})'
                           for n, x, (lo, hi) in zip(NAMES, v[:6], limits) if not lo <= x <= hi]
                    print('\nno jog: outside the motor limits, the board would ignore every command: ' + ', '.join(out)
                          + '. Support the arm, r, move it inside by hand, h, then j.')
                else:
                    jog['target'] = list(v[:6])
                    jog['on'] = True
                    print(f'\njog on from here: axis {NAMES[jog["axis"]]}, step {steps_of(jog["axis"])[jog["step"]]:g} deg')
            elif k in '123456':
                jog['axis'] = int(k) - 1
                jog['step'] = min(jog['step'], len(steps_of(jog['axis'])) - 1)
                print(f'\naxis {k}: {NAMES[jog["axis"]]}')
            elif k in '[]':
                jog['step'] = max(0, min(len(steps_of(jog['axis'])) - 1, jog['step'] + (1 if k == ']' else -1)))
                print(f'\nstep {steps_of(jog["axis"])[jog["step"]]:g} deg')
            elif k in '+=-' and not jog['on']:
                print('\njog is off: j first')
            elif k in '+=-' and jog['on']:
                a, d = jog['axis'], math.radians(steps_of(jog['axis'])[jog['step']]) * (-1 if k == '-' else 1)
                lo, hi = limits[a]
                jog['target'][a] = min(max(jog['target'][a] + d, lo), hi)
            elif k == 'u' and saved:
                print(f'\ndropped {saved.pop()["name"]}')
                yaml.safe_dump(saved, open(OUT, 'w'), sort_keys=False)
            elif k in '\r\n' and v is not None:
                raw = [round(math.degrees(x), 1) for x in v[:6]]
                saved.append({'name': nxt, 'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'raw_deg': raw,
                              'target': [round(x, 5) for x in raw_to_target(raw)],
                              'jog': jog['on']})
                yaml.safe_dump(saved, open(OUT, 'w'), sort_keys=False)
                names = names[1:]
                print(f'\nsaved {nxt}: ' + '  '.join(f'{n} {x}' for n, x in zip(NAMES, raw)))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        stop.set()
        time.sleep(0.1)
        v = latest[0]
        print(f'\n{len(saved)} poses in {OUT}.'
              + (' Torque is still OFF: support the arm, then holdarm.'
                 if v is not None and int(v[6]) == 2 and not jog['on'] else ''))
        s.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
