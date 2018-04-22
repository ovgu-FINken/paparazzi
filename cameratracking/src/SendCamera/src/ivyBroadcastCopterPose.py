#!/usr/bin/env python
import math
from IvyBroadcastNode import *
from ivy.std_api import *


comm=IvyBroadcastNode()
comm.IvyInitStart()
comm.IvyBindINS()

try:
    while True:
        #do nothing
        pass
        #print('test')
except KeyboardInterrupt:
    print('\nROS initialization canceled by user')

comm.IvyInitStop()



