#!/bin/bash

dmesg | grep pl2303 | grep attached | tail -1 | cut -f9 -d" "
