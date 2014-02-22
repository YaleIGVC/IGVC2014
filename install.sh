#!/bin/bash
sudo apt-get install ros-hydro-hector-mapping ros-hydro-hector-slam ros-hydro-hokuyo-node ros-hydro-serial ros-hydro-libsegwayrmp

catkindirpath="~/catkin_workspace/src"
username=`whoami`

# Add ourselves to dialout so that we don't need to chmod USB devices
sudo usermod -aG dialout $username

mkdir -p $catkindirpath
cd $catkindirpath

catkin_init_workspace
catkin_make
git clone https://github.com/segwayrmp/segway-rmp-ros-pkg.git
cd segway-rmp-ros-pkg
