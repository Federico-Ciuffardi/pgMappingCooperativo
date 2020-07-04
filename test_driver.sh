#! /usr/bin/sh

echo "> Compiling"
echo ""

cd $HOME/catkin_ws
catkin_make
echo ""

# segunda vuelta

$HOME/catkin_ws/src/tscf_exploration/test_once.sh "office" "5"

