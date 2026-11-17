#!/usr/bin/env python3
"""Reject malformed CLI inputs without starting ROS or opening hardware."""
from pathlib import Path
import subprocess
import sys

cli = Path(sys.argv[1]).resolve()
cases = [
    ['--lie', '--cell', '2oops', '--pick'],
    ['--lie', '--cell', '2.5', '--pick'],
    ['--find', '--slot', '8oops'],
    ['--find', '--slot', '12,'],
    ['--find', '--lm-tol-mm', '2oops'],
    ['--find', '--hold-before-close', '0x1p2'],
]
for args in cases:
    result = subprocess.run([str(cli), '--dry-run', *args], capture_output=True, text=True)
    assert result.returncode != 0, (args, result.stdout, result.stderr)
for args in [
    ['--planning', '--collect', '--expected-pucks', '2', '--box', 'S1', '--slot', '12', '--auto'],
    ['--planning', '--find', '--box', 'S1', '--slot', '11', '--auto'],
    ['--planning', '--lie', '--cell', '2', '--pick', '--box', 'S1', '--slot', '12', '--auto'],
    ['--planning', '--lie', '--cell', '5', '--pick', '--box', 'S1', '--slot', '12', '--auto'],
    ['--find', '--slot', '1_2', '--lm-tol-mm', ' 20.0 '],
]:
    subprocess.run([str(cli), '--dry-run', *args], capture_output=True, text=True, check=True)
print('CLI validation: 6 malformed arguments rejected, 5 valid dry runs passed')
