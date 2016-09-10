# turtle_draw

A ROS node for [ROS Kinetic Kame](http://wiki.ros.org/kinetic) that draws geometric figures using [turtlesim](http://wiki.ros.org/turtlesim).

**NOTE:** the following scripts use relative paths, and will only work if they are run from this directory.

## Installation

Run `install_ros_kinetic.sh` to install ROS Kinetic Kame. Alternatively, you could follow the [official instructions](http://wiki.ros.org/kinetic/Installation). The `setup.bash` script is not added to `.bashrc`, so you'll need to manually run `source /opt/ros/kinetic/setup.bash` to set up the environment.

## Compilation

Run `compile.sh`.

## Usage

Run `run-turtle-draw.sh` to start the node. The roscore must have been previously started by running `roscore`.
