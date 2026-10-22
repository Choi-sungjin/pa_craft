#!/usr/bin/env python3
"""Compatibility launcher for the C++ hierarchical PlanningMaster."""
import os
import sys
if __name__ == '__main__':
    os.execvp('ros2', ['ros2', 'run', 'craft_planning', 'planning_master', *sys.argv[1:]])
