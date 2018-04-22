#!/usr/bin/env python

from IvyCalibrationNode import *
from ivy.std_api import *

comm=IvyCalibrationNode()
comm.IvyInitStart()

comm.IvySendCalib(5, 58, -0.005) #Roll Left (Negative), Right (Positive)
comm.IvySendCalib(5, 59, 0.010) #Pitch Forward (Negative), Backward (Positive)

comm.IvyInitStop()
