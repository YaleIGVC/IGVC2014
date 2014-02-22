#!/bin/bash
sudo apt-get install ros-hydro-hector-mapping ros-hydro-hector-slam ros-hydro-hokuyo-node

catkindirpath="~/catkin_workspace/src"
mkdir -p $catkindirpath
cd $catkindirpath

catkin_init_workspace
catkin_make
git clone https://github.com/segwayrmp/segway-rmp-ros-pkg.git
cd segway-rmp-ros-pkg
