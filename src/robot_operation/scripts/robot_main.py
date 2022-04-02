#!/usr/bin/env python3

from robot_lib import *

r = Robot()
microcontrollers = r.detectMicrocontrollers()
accesories  = r.detectAccesories()
r.calibrate()