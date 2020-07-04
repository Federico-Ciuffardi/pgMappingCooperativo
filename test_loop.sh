#! /usr/bin/sh

echo "> Compiling"
echo ""

cd $HOME/catkin_ws
catkin_make
echo ""

echo "> Running Roscore"
echo ""
roscore &>/dev/null &

sleep 30

map_name="office"
starting_robot_number="5"

while true ; do
	pkill blender -9

	echo "> Simulation on $map_name with $starting_robot_number"
	echo ""

	echo "> Running morse"
	echo ""
	morse run ~/catkin_ws/src/tscf_exploration/morse/$map_name$starting_robot_number.py &>/dev/null &

	sleep 30

	echo "> Runing roslaunch"
	echo ""
	roslaunch tscf_exploration tscf_multirobot.launch starting_robot_number:=$starting_robot_number map_name:=$map_name 1>/dev/null #2>test_roslaunch.log
done
