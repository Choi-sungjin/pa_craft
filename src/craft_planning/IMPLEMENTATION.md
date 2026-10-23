# Planning / Control execution contract

Planning owns SystemState (`Idle`, `Ready`, `Run`, `Fault`) and optional RunPhase
(`AwaitRequest`, `PrepareTask`, `Compute`, `Execute`, `Complete`, `Stopping`).
ControlMaster owns task distribution, controller completion and measured-stop aggregation.
The `craft_interfaces` package is their shared protocol; no cyclic package dependency.

- Queue requests in Run, assign monotonic runtime task IDs on enqueue, and execute FIFO.
- START enters Run from Ready. CANCEL clears work and waits for stop before AwaitRequest.
  STOP clears work and waits for stop before Ready. RESET requires measured stop, reinitializes
  configured planners, and returns through Idle. Startup never auto-starts a mission.
- TaskCommand includes the existing path, ordered IK joints, gripper, slot or wait payload.
  ControlMaster selects participants; Planning does not send per-controller commands.
- TaskResult repeats task ID, revision, status, reason and `[Arm,LM,Wheel,Clock,Gripper]` matrix.
  Only the current dispatched ID affects Planning. Duplicate/late messages cannot advance another task.
- Controller arrival plus measured settling determines completion. Planning does not recalculate
  wheel distance, clock slot, motor arrival or stop state. Diagnostic Bool topics are not consumed.
- Stops need fresh ControlStatus held+stopped and a matching STOPPED result if a task was dispatched.
  Stop timeout becomes Fault; a failed task cannot subsequently become successful.
- Navigation regenerates full paths periodically using current pose and obstacle snapshots.
  Updates preserve task ID, advance revision, and remain in Execute. Control preserves path segments.
  A temporarily unavailable route pauses wheel speed and retries with the task and queue intact.
  Invalid or expired replans fault. No separate FSM collision-monitor phase exists.
- Deadlines are monotonic. ROS acquisition times and receive watchdogs reject stale inputs.
  Asynchronous computations use generations; cancelled/faulted results are discarded.
- Motor protocol, calibration ownership and OpenCR watchdog behavior remain in Control/firmware.
  No firmware task IDs are needed: controllers are in-process and ControlMaster owns the active row.

Build and test with matching ROS/Pinocchio dependencies:

```bash
colcon build --packages-up-to craft_planning craft_control --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select craft_planning craft_control --event-handlers console_direct+
colcon test-result --verbose
```

Tests exercise the pure FSM, controller matrix, map/FK/IK/configuration modules, synthetic
ControlMaster feedback, disposable PTY boards, and both real masters over synthetic sensors
and PTYs. They do not verify physical motor calibration or real robot operation.
See [README.md](README.md) and [control README](../craft_control/README.md) for operational commands.
