#!/bin/bash

segway_idProduct=`lsusb | grep Segway | cut -f6 -d" " | cut -f2 -d":"`
segway_usbno=`dmesg | grep $segway_idProduct | cut -f5 -d" " | cut -f1 -d":"`
SEGWAY_TTY=`dmesg | grep $segway_usbno | grep ttyUSB | cut -f14 -d" "`

vectornav_idProduct=`lsusb | grep FT232 | cut -f6 -d" " | cut -f2 -d":"`
vectornav_usbno=`dmesg | grep $vectornav_idProduct | cut -f5 -d" " | cut -f1 -d":"`
VECTORNAV_TTY=`dmesg | grep $vectornav_usbno | grep ttyUSB | cut -f14 -d" "`

gps_idProduct=`lsusb | grep PL2303 | cut -f6 -d" " | cut -f2 -d":"`
gps_usbno=`dmesg | grep $gps_idProduct | grep idProduct | cut -f7 -d" " | cut -f1 -d":"`
GPS_TTY=`dmesg | grep $gps_usbno | grep ttyUSB | cut -f13 -d" "`


ports_path=~/IGVC2014/launch_files/rosparams/ports.yaml
if [ -e $ports_path ]
then
    rm $ports_path
fi

echo "segway_serial_port: /dev/$SEGWAY_TTY" >> $ports_path
echo "vectornav_serial_port: /dev/$VECTORNAV_TTY" >> $ports_path
echo "gps_serial_port: /dev/$GPS_TTY" >> $ports_path
