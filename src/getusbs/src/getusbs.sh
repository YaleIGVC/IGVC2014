#!/bin/bash

segway_idProduct=`lsusb | grep Segway | cut -f6 -d" " | cut -f2 -d":"`
segway_usbno=`dmesg | grep $segway_idProduct | grep -oe '[0-9]-[0-9]'`
SEGWAY_TTY=`dmesg | grep $segway_usbno | grep -oe 'ttyUSB[0-9]'`

vectornav_idProduct=`lsusb | grep FT232 | cut -f6 -d" " | cut -f2 -d":"`
vectornav_usbno=`dmesg | grep $vectornav_idProduct | grep -oe '[0-9]-[0-9]'`
VECTORNAV_TTY=`dmesg | grep $vectornav_usbno | grep -oe 'ttyUSB[0-9]'`

# Commented out because we do not yet have a GPS installed
#gps_idProduct=`lsusb | grep PL2303 | cut -f6 -d" " | cut -f2 -d":"`
#gps_usbno=`dmesg | grep $gps_idProduct | grep idProduct | grep -oe '[0-9]-[0-9]'`
#GPS_TTY=`dmesg | grep $gps_usbno | grep ttyUSB | cut -f13 -d" "`

export SEGWAY_SERIAL_PORT=/dev/$SEGWAY_TTY
export IMU_SERIAL_PORT=/dev/$VECTORNAV_TTY
export GPS_SERIAL_PORT=/dev/$GPS_TTY
