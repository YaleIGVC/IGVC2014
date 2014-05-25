#!/bin/bash

source ~/.bashrc
# Modify this line to change what file to launch
LAUNCHFILE_NAME=~/IGVC2014/launch_files/test.launch

# Get environment files from getsusbs
file=~/IGVC2014/src/getusbs/src/getusbs.sh
source $file

roslaunch $LAUNCHFILE_NAME

