#!/bin/bash

# First we need to add packages.ros.org to the list of package sources

# Only Wily (Ubuntu 15.10), Xenial (Ubuntu 16.04) and Jessie (Debian 8)
# are supported, so if we're not running one of those, the best we can
# do is pose as one of those

echo "wily xenial jessie" | grep -i $(lsb_release -sc) >/dev/null

if [ $? -eq 0 ]; then
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
else
    POSING_DISTRO="xenial" # This assumes an Ubuntu 16.04 based dsitribution
    echo "WARN: you're not running one of the distributions supported by ROS, posing as $POSING_DISTRO. This might cause issues"

    ROS_APT_SOURCE="\"deb http://packages.ros.org/ros/ubuntu $POSING_DISTRO main\""
    echo $ROS_APT_SOURCE
    sudo sh -c "echo $ROS_APT_SOURCE > /etc/apt/sources.list.d/ros-latest.list"
fi

# Add the server's public key
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

# Download the full package
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full -y

# Initialize rosdep
sudo rosdep init
rosdep update

echo "Successfully installed ROS Kinetic Kame!"
