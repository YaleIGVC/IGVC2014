#!/bin/bash

# Modify this line to change what file to launch
LAUNCHFILE_NAME=test.launch

# Get environment files from getsusbs
dir=`roscd getusbs`
file=$dir/src/getusbs.sh
source $file

roslaunch $LAUNCHFILE_NAME

