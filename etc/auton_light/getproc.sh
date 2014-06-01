#!/bin/bash

output=`ps aux | grep image_unwarp | cut -f1 -d$'\n' | grep grep`
echo $output
