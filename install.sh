#!/bin/bash
sudo apt-get install ros-hydro-hector-mapping ros-hydro-hector-slam ros-hydro-hokuyo-node ros-hydro-serial ros-hydro-libsegwayrmp nautilus-dropbox python-pip
sudo pip install pyserial

echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
echo "source ~/catkin_workspace/devel/setup.bash" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_workspace" >> ~/.bashrc

catkindirpath="~/catkin_workspace/src"
username=`whoami`

# Add ourselves to dialout so that we don't need to chmod USB devices
sudo usermod -aG dialout $username

mkdir -p $catkindirpath
cd $catkindirpath

catkin_init_workspace
cd ..
catkin_make
cd src
git clone https://github.com/segwayrmp/segway-rmp-ros-pkg.git
cd segway-rmp-ros-pkg
