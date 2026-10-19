// wheel: target_angular_velocity / LM,Arm,Clock: target_angle
// feedback: motor_feedback, arm_feedback, rotation feedback
// TODO: arm+LM OpenCR frame order must be slide,shoulder,elbow,wrist_pitch,tool_roll,gripper.
// TODO: arm+LM feedback = these six positions, then arm_lm_state, then measured gripper_current_a [A].
// TODO: clock+wheel feedback = wheel_l, wheel_r, clock, then wheel_state, clock_state.
