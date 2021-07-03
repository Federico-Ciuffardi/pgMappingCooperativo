#! /usr/bin/sh

echo "> Compiling"
echo ""

cd $HOME/catkin_ws
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 || exit 1
echo ""

# segunda vuelta

$HOME/catkin_ws/src/pgmappingcooperativo/test_once.sh "office" "2"

