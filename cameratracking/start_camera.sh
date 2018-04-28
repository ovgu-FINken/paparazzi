#!/bin/bash

# put dns entry into hostfile as we sometimes do not find brain
# echo "192.168.147.151 brain" > /etc/hosts


# source needed files
source /opt/ros/kinetic/setup.bash
source devel/setup.bash

# kill existing ros
pkill rosmaster
pkill roscore

# start local roscore
roscore -p 11311 &
sleep 3s

# log in to brain and start camera node
ssh parlow@brain './start_camera' &

# start camera node
rosrun send_camera start_camera_node

