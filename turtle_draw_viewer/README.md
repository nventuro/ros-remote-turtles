# turtle_draw_viewer

A simple webapp that connects to a local turtlesim instance using [rosbridge](http://wiki.ros.org/rosbridge_suite) and draws on a browser whatever turtlesim is drawing.

**NOTE:** the following scripts use relative paths, and will only work if they are run from this directory.

## Installation

Run `install-rosbridge.sh` to install rosbridge for ROS Kinetic Kame. ROS must have been previously installed for this step to work (you can use `turtle_draw/install-ros-kinetic.sh` to install ROS).

## Usage

For the webapp to work, ROS and rosbridge must be running locally. Specifically, the webapp attempts to connect to a WebSocket server at `ws://localhost:9090`. To run it, simply load `index.html` on a web browser.
