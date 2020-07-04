#! /usr/bin/sh

map_name=$1
starting_robot_number=$2

pkill roscore
pkill blender -9
pkill rqt_console
pkill rviz

date
echo "> Running Roscore"
echo ""

roscore &>/dev/null &

sleep 30

rviz &>/dev/null &
sleep 1

rqt_console &>/dev/null &
sleep 1

echo "> Simulation on $map_name with $starting_robot_number"
echo ""

echo "> Running morse"
echo ""
morse run ~/catkin_ws/src/tscf_exploration/morse/$map_name$starting_robot_number.py &>/dev/null &

sleep 30

echo "> Running roslaunch"
echo ""
roslaunch tscf_exploration tscf_multirobot.launch starting_robot_number:=$starting_robot_number map_name:=$map_name &>/dev/null #2>test_roslaunch.log

