#! /usr/bin/env bash

map_name=$1
starting_robot_number=$2

pkill roscore
pkill rqt_console
pkill rviz

date
echo "> Running Roscore"
echo 

roscore &>/dev/null &

sleep 5

rviz -d /home/fede/catkin_ws/src/pgmappingcooperativo/rviz/"$map_name".rviz &>/dev/null &
rqt_console &>/dev/null &

echo "> Simulation on $map_name with $starting_robot_number"
echo
roslaunch pgmappingcooperativo multirobot.launch starting_robot_number:=$starting_robot_number map_name:=$map_name #&>/dev/null #2>test_roslaunch.log

