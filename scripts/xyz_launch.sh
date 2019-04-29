#!/usr/bin/env bash

#
echo "Run script: get_xyz_service.py"
rosrun robot_control get_xyz_service.py joint_states:=/robot/joint_states &

echo "Run script: goto_xyz_service.py"
rosrun robot_control goto_xyz_service.py joint_states:=/robot/joint_states &
