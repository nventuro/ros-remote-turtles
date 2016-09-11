#!/usr/bin/env bash

# turtle_draw.py receives command line arguments, but the ROS bash scripts
# attempt to use $@, so we need to store the arguments in a separate variable
# and then delete them from $@

ARGS="$@"
for arg in "$@" ; do
    shift
done

source /opt/ros/kinetic/setup.bash
source catkin_ws/devel/setup.bash

rosrun turtle_draw turtle_draw.py $ARGS
