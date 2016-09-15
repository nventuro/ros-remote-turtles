#!/usr/bin/env bash

source /opt/ros/kinetic/setup.bash

rostopic pub /turtle_draw/run std_msgs/Bool True
