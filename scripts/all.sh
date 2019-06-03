#!/usr/bin/env bash

echo "Start Joint Tragectory Service"
rosrun intera_interface joint_trajectory_action_server.py &

echo "Start gripper server"
rosrun robot_control open_close_gripper_server.py &

cd ../../sawyer_moveit

./sawyer_clicksmart_moveit_launch.sh &
# Start Custom Services

echo "Start Service: goto_xyz"
rosrun robot_control goto_xyz_service.py joint_states:=/robot/joint_states &

echo "Start Service: manipulate_head_service"
rosrun robot_control get_gripper_status_service.py &

echo "Start Service: move_arm_out_of_way_service"
rosrun robot_control move_arm_out_of_way_service.py &
