# turtle_draw_control

A simple Android application that connects to a remote [turtle_draw](https://github.com/nventuro/ros-remote-turtles/tree/master/turtle_draw) instance using [rosbridge](http://wiki.ros.org/rosbridge_suite), and controls the flow of the node by sending run/pause commands.

**NOTE:** the following scripts use relative paths, and will only work if they are run from this directory.

## Installation

This depends on [rosjava's android_core project](https://github.com/rosjava/android_core), which must be cloned (`git clone https://github.com/rosjava/android_core`) beforehand. The application was tested using the `indigo` branch (`git checkout indigo`). Once android_core is present on the system, `install.sh` can be run, which will create a symlink to this project inside of the android_core directory.

The application is built using Android Studio. The android_core project should be imported, which will also import all tutorials and this project (because of the symlink). android_core's gradle build system must be modified to include turtle_draw_control:

 * line 32 of `android_core/build.grade` needs to be changed to

`configure(subprojects.findAll { it.name.startsWith("android_") || (it.name == "turtle_draw_control") }) {`

 * `include "turtle_draw_control"` needs to be added to `android_core/settings.gradle`

After that, the application can be successfully built with no further changes.
