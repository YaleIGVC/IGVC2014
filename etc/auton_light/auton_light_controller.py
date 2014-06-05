#!/usr/bin/env python

import serial
import os

s = serial.Serial('/dev/ttyACM0',9600)

ps = os.popen('~/IGVC2014/etc/auton_light/auton_light_controller.py')

while True:

