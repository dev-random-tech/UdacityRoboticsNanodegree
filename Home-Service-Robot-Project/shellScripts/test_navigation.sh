#!/bin/sh
xterm -e " roslaunch turtlebot3_gazebo turtlebot3_world.launch world_file:=/home/dev/catkin_ws/src/world/my_world.world " &
sleep 5
xterm -e " roslaunch turtlebot3_gazebo amcl_demo.launch map_file:=/home/dev/catkin_ws/src/world/mymap.yaml " &
sleep 5
xterm -e " roslaunch turtlebot3_rviz_launchers view_navigation.launch " &


