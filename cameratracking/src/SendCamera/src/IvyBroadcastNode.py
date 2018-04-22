#!/usr/bin/python2
import math
from ivy.std_api import *
import time
"""kill_log resides in the upper directory to this script"""
import os
parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.sys.path.insert(0,parentdir)
import kill_log

# list of copters that send INS data to this node, that's why we publish INS data from other copters to them.
signed_copters = []

class IvyBroadcastNode:
    def __init__(self):
        self.myKillLog = kill_log.KillLog()
    
    def IvyInitStart(self):
        """ Initializes the Ivy Server and ROS Subscriber

        Should only be called once per session.
        """
	

        try:
            IvyInit('Broadcast Node', '', 0)
        except AssertionError:
            print('Assertion Error in IvyInit(!= none), is there a server already running? Exiting')
            IvyStop()
            raise SystemExit()
        IvyStart()
        time.sleep(1)
        print('Ivy Broadcast Node ready!')


    def IvyBindINS(self):
	IvyBindMsg(self.handleINSdata, '(.* INS .*)')

    def handleINSdata(self,agent, data):
	global signed_copters
	# (toid, asid)
        #self.IvySendGPSBroadcast(3, 5, 384205230, 79184980, 501233200, 0, 0, 0)
	#IvySendMsg('3 COPTER_FORCE 0.5 0.3')
	#array = data.split(" ")
	#copter_id = int(array[0])
	#ins_x = int(array[2])
	#ins_y = int(array[3])
	#ins_z = int(array[4])
	#ins_xd = int(array[5])
	#ins_yd = int(array[6])
	#ins_zd = int(array[7])
	#ins_xdd = int(array[8])
	#ins_ydd = int(array[9])
	#ins_zdd = int(array[10])
	#if copter_id not in signed_copters:
	#	signed_copters.append(copter_id)
	#print(signed_copters, copter_id, ins_x, ins_y, ins_z, ins_xd, ins_yd, ins_zd, ins_xdd, ins_ydd, ins_zdd)
	#for ac_id in signed_copters:
	#	if (ac_id != copter_id):
	#		print("Sending to ", ac_id)
	#		self.IvySendINSBroadcast(ac_id, copter_id, ins_x, ins_y, ins_z, ins_xd, ins_yd, ins_zd, ins_xdd, ins_ydd, ins_zdd)		
	

    def IvyInitStop(self):
        """Stops the Ivy Server.
        """
        time.sleep(1)
        IvyStop()

    def IvyGetPos(self):
        """Simply returns the position grabbed via ROS to the caller

        """
        try:
            return copterPos
        except NameError as e:
            print("CopterPos not yet defined! (No data from ROS?):")
            print(str(e))

    def IvyGetPosList(self):
        """Returns the position to a list for less dependency with ros
           Returns
           -------
           List
        """
        position = self.IvyGetPos()
        return [position.x, position.y, position.theta]

    def IvySendCalib(self,param_ID, AC_ID, value):
        """Sends the given parameter via Ivy

        AC_ID:      ID of the aricraft to be calibrated
        param_ID:   ID of the parameter to be calibrated
                     phi   = 58 roll
                     theta = 59 pitch
                     psi   = 60 yaw
        value:      value to be set for the parameter !in degrees!
        """
        print("sending calib msg")
        IvySendMsg('dl SETTING %d %d %f' %
                    (AC_ID,
                    param_ID,
                    value
                    ))


    def IvySendKill(self, AC_ID):
        """Sends a kill message to the aircraft

        """
        IvySendMsg('dl KILL %d 1' %
                    (AC_ID
                    ))
        
    def SetInDeadZone(self,inDeadZone):
        self.myKillLog.inDeadZone = inDeadZone

    def IvySendCalParams(self, AC_ID, turn_leds, roll, pitch, yaw):
        IvySendMsg('dl CALPARAMS %d %d %f %f %f' %
                    (AC_ID, turn_leds, roll, pitch, yaw
                    ))

    def IvySendCopterPose(self, AC_ID, posX, posY, theta):
        IvySendMsg('dl COPTERPOSE %d %f %f %f' %
                    (AC_ID, posX, posY, theta
                    ))


    def IvySendRemoteGPS(self, AC_ID, numsv, ecef_x, ecef_y, ecef_z, lat, lon, alt, hmsl, ecef_xd, ecef_yd, ecef_zd, tow, course):
        IvySendMsg('dl REMOTE_GPS %d %d %d %d %d %d %d %d %d %d %d %d %d %d' %
                    (AC_ID, numsv, ecef_x, ecef_y, ecef_z, lat, lon, alt, hmsl, ecef_xd, ecef_yd, ecef_zd, tow, course
                    ))

    def IvySendINSBroadcast(self, AC_ID, copter_id, ins_x, ins_y, ins_z, ins_xd, ins_yd, ins_zd, ins_xdd, ins_ydd, ins_zdd):
        IvySendMsg('dl COPTER_INS %d %d %d %d %d %d %d %d %d %d %d' %
                    (AC_ID, copter_id, ins_x, ins_y, ins_z, ins_xd, ins_yd, ins_zd, ins_xdd, ins_ydd, ins_zdd
                    ))

    def IvySendGPSBroadcast(self, AC_ID, copter_id, gps_x, gps_y, gps_z, gps_xd, gps_yd, gps_zd):
        IvySendMsg('dl COPTER_GPS %d %d %d %d %d %d %d %d' %
                    (AC_ID, copter_id, gps_x, gps_y, gps_z, gps_xd, gps_yd, gps_zd
                    ))

    def IvySendUnKill(self, AC_ID):
        """Sends an unkill message to the aircraft

        """
        IvySendMsg('dl KILL %d 0' %
                    (AC_ID
                    ))


    def IvySendSwitchBlock(self, AC_ID, block_ID):
        """Sends a message to switch the flight plan

        """
        IvySendMsg('dl BLOCK %d %d' %
                    (block_ID,
                     AC_ID,
                     ))
                     
    def SaveIvyKillLog(self):
        self.myKillLog.saveLog()
