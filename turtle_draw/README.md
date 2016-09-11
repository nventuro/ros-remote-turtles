# turtle_draw

A ROS node for [ROS Kinetic Kame](http://wiki.ros.org/kinetic) that draws geometric figures using [turtlesim](http://wiki.ros.org/turtlesim).

**NOTE:** the following scripts use relative paths, and will only work if they are run from this directory.

## Installation

Run `install_ros_kinetic.sh` to install ROS Kinetic Kame. Alternatively, you could follow the [official instructions](http://wiki.ros.org/kinetic/Installation). The `setup.bash` script is not added to `.bashrc`, so you'll need to manually run `source /opt/ros/kinetic/setup.bash` to set up the environment.

## Compilation

Run `compile.sh`. Since the node is written in Python, the compilation step needs only be executed once, so that the ROS package can be built.

## Usage

For the node to work, both the roscore and the turtlesim node must have been previously started by running `roscore` and `rosrun turtlesim turtlesim_node` (from a terminal where `/opt/ros/kinetic/setup.bash` has been `source`'d). Run `run-turtle-draw.sh` to start the node. To see all available options, run `run-turtle-draw.sh --help`.

A particularly intersting setup is `./run-turtle-draw.sh --figure figures/star.figure --turtle-army`.

### Figure files
Figures are specified in plaintext files, where each line is an (x, y) pair (comma-separated), where the range for both the x and y coordinates is [-1, 1]. The turtle will travel from point to point in the same order they are in the file. Some examples can be found in the `figures` directory. Not much error checking is performed on the figure files, so take care to make sure the contents are correct.
