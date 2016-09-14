# turtle_draw_viewer

A simple webapp that connects to a local turtlesim instance using [rosbridge](http://wiki.ros.org/rosbridge_suite) and draws on a browser whatever turtlesim is drawing.

**NOTE:** the following scripts use relative paths, and will only work if they are run from this directory.

## Installation

Run `install-rosbridge.sh` to install rosbridge for ROS Kinetic Kame. ROS must have been previously installed for this step to work (you can use `../turtle_draw/install-ros-kinetic.sh` to install ROS).

## Usage

For the webapp to work, the roscore and rosbridge must be running locally. Specifically, the webapp attempts to connect to a WebSocket server at `ws://localhost:9090`. This can be achieved by running `../misc/run-roscore.sh` and then `run-rosbridge.sh` (in separate terminals). To run the webapp, simply load `index.html` on a web browser. If rosbridge was not running when the webapp was first loaded, it needs to be reloaded by the browser in order to connect to the WebSocket server.
