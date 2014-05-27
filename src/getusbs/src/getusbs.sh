#!/bin/bash

segway_idProduct=`lsusb | grep Segway | cut -f6 -d" " | cut -f2 -d":"`
segway_usbno=`dmesg | grep $segway_idProduct | tail -1 | grep -oe '[0-9]-[0-9]'`
SEGWAY_TTY=`dmesg | grep $segway_usbno | tail -1 | grep -oe 'ttyUSB[0-9]'`

vectornav_idProduct=`lsusb | grep FT232 | cut -f6 -d" " | cut -f2 -d":"`
vectornav_usbno=`dmesg | grep $vectornav_idProduct | grep -oe '[0-9]-[0-9]' | tail -1 `
VECTORNAV_TTY=`dmesg | grep $vectornav_usbno | grep -oe 'ttyUSB[0-9]' | tail -1 `

gps_idProduct=`lsusb | grep PL2303 | cut -f6 -d" " | cut -f1 -d":"`
gps_usbno=`dmesg | grep $gps_idProduct | grep idProduct | tail -1 | grep -oe '[0-9]-[0-9]'`
GPS_TTY=`dmesg | grep $gps_usbno | grep ttyUSB | cut -f13 -d" "`

export SEGWAY_SERIAL_PORT=/dev/$SEGWAY_TTY
export IMU_SERIAL_PORT=/dev/$VECTORNAV_TTY
export GPS_SERIAL_PORT=/dev/$GPS_TTY

echo Segway serial port is $SEGWAY_SERIAL_PORT
echo IMU serial port is $IMU_SERIAL_PORT
echo GPS serial port is $GPS_SERIAL_PORT
