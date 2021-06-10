#!/usr/bin/env python

import sys
import time
from os import path, getenv

PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python") # pprzlink

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

import matplotlib.pyplot as plt
import numpy as np

aircrafts = {}
msg_class = "telemetry" 
data = None

def message_recv(ac_id, msg):
    """Handle incoming messages

    Callback function for IvyMessagesInterface

    :param ac_id: aircraft id
    :type ac_id: int
    :param msg: message
    :type msg: PprzMessage
    """
    # only show messages of the requested class
    if msg.msg_class != msg_class:
        return
    if ac_id in aircrafts and msg.name in aircrafts[ac_id].messages:
        if time.time() - aircrafts[ac_id].messages[msg.name].last_seen < 0.2:
            return
    if msg.name == "LEADER":
        global data
        dataF = np.array([msg['floats']], dtype=float)
        dataI = np.array([msg['ints']], dtype=np.int32)
        dataC = np.array([msg['strings']], dtype=str)
        print(dataF, dataI, dataC)

def main():
    fig = plt.figure()
    plt.show()
    interface = IvyMessagesInterface("Paparazzi Messages Viewer")
    interface.subscribe(message_recv)

if __name__ == '__main__':
    main()
