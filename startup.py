#!/usr/bin/python

from subprocess import call

call("roscore &", shell=True)
call("~/ros/joyRateModifier.py &", shell=True)
call("rosrun rosserial_python serial_node.py /dev/ttyACM0 &", shell=True)
