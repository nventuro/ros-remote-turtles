#!/usr/bin/env bash

source ../misc/add-packages.sh

# Download the full package
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full -y

# Initialize rosdep
sudo rosdep init
rosdep update

echo "Successfully installed ROS Kinetic Kame!"
