#!/bin/bash

roscore &
CORE_PID=$!

sleep 5

# ssh -t -t turtlebot2@192.168.1.101 << HERE
#  source /opt/ros/indigo/setup.bash
#  /home/ross/Documents/computerVision/finalProject/kinect_to_arena_map_tradefair.py
#  sleep 3
#  exit
# HERE

ssh -t -t turtlebot2@192.168.1.101 << HERE
  source /opt/ros/indigo/setup.bash
  roslaunch /home/turtlebot2/turtlebot_bringup/launch/minimal.launch &
  sleep 20
  exit
HERE

ssh -t -t turtlebot2@192.168.1.101 << HERE
  source /opt/ros/indigo/setup.bash
  roslaunch /home/turtlebot2/turtlebot_navigation/launch/amcl_demo.launch map_file:=/home/turtlebot2/arena_img.yaml initial_pose_x:=1.5 initial_pose_y:=.368 initial_pose_a:=1.57 &
  sleep 20
  exit
HERE

roslaunch turtlebot_rviz_launchers view_navigation.launch --screen &
RVIZ_PID=$!

sleep 20

# rosrun simple_navigation_goals simple_navigation_goals
# NAVIGATION_PID=$!

wait $RVIZ_PID

# echo "NAVIGATION COMPLETE"

# kill $RVIZ_PID

ssh -t -t turtlebot2@192.168.1.101 << HERE
  kill \$(ps aux | grep '[a]mcl_demo' | awk '{print \$2}')
  kill \$(ps aux | grep '[m]inimal.launch' | awk '{print \$2}')
  exit
HERE

kill $CORE_PID

