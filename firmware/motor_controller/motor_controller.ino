// wheel: target_angular_velocity / LM,Arm,Clock: target_angle
// feedback: motor_feedback, arm_feedback, rotation feedback
// TODO: arm+LM OpenCR frame order must be slide,shoulder,elbow,wrist_pitch,tool_roll,gripper.
// TODO: arm feedback must append measured gripper_current_a [A] after these six positions.
