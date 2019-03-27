#!/usr/bin/env bash

set -ex

# Install ros and setup ros environment
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-ros-base

source /opt/ros/kinetic/setup.sh

sudo apt-get install -y python3 python3-pip python3-apt

# Run rosdep init only once
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

sudo sh -c 'echo "yaml https://s3-us-west-2.amazonaws.com/rosdep/base.yaml" > /etc/ros/rosdep/sources.list.d/19-aws-sdk.list'
rosdep update

rosdep install --from-paths integration/test_workspace --as-root=pip:false --ignore-src -r -y

pip3 install --upgrade pip setuptools
pip3 install -e .

cd integration/test_workspace
colcon build
