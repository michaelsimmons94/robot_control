# sawyer_gripper
Rosservice Calls:

HEAD:
 - head/face_forward
 - head/pan_to_angle

ARM:
 - arm/move_away
 - arm/return_to_position
 - arm/move_to_rest
 - arm/get_xyz
 - arm/goto_xyz [x] [y] [z]

GRIPPER:
 - gripper/actuate [open || close}
 - gripper/status
