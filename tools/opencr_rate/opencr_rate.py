#!/usr/bin/env python3
"""Feedback rate of an OpenCR board, read straight from its port (2026-11-12: the wheel board sent 36 Hz, not 50).
Stack down (control_master holds the port):  /usr/bin/python3 opencr_rate.py [/dev/opencr_wheel_clock] [seconds]"""
import sys, time
import numpy as np
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/opencr_wheel_clock"
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
with serial.Serial(port, 115200, timeout=0.1) as s:
    s.reset_input_buffer()
    t, end = [], time.time() + seconds
    while time.time() < end:
        line = s.readline()
        if line and not line.startswith(b"D,"):  # feedback lines only, not the 1 s diagnostic
            t.append(time.perf_counter())
if len(t) < 3:
    sys.exit(f"{port}: {len(t)} feedback lines in {seconds:.0f} s (board off, hung, or still in its 5 s IMU start)")
g = np.diff(t) * 1000
print(f"{port}: {len(t) / seconds:.1f} Hz, gap median {np.median(g):.1f} ms (p95 {np.percentile(g, 95):.1f}); 50 Hz = 20 ms")
