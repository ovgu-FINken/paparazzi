#!/usr/bin/python2
import rospy
import math
from ivy.std_api import *
from std_msgs.msg import String
import rospy
import geometry_msgs.msg
import math
from ivy.std_api import *
#from geometry_msgs.msg import Pose2D
from tracking.msg import TaggedPose2D
import time
"""kill_log resides in the upper directory to this script"""
import os
parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.sys.path.insert(0,parentdir)
import kill_log

# global values
deg2rad = math.pi / 180
rad2deg = 180 / math.pi
lat = 52.138821 * deg2rad  # Latitude of FIN
lon = 11.645634  * deg2rad # Longitude of FIN
gps_lat = 52.13882 * 10000000
gps_lon = 11.645634 * 10000000
weekInMilliseconds = 604800000

# evaluation data
virt_pos_x = 384205230
virt_pos_y = 79185080
virt_pos_z = 501233200

wall_top_x = 384205148 
wall_bottom_x = 384205341 
wall_left_y = 79184823
wall_right_y = 79185130 
safe_dist = 40
wall_hit_count = 0
wall_was_hit = False

class OldData:
    def __init__(self):
	self.time = rospy.get_rostime()
	self.data = TaggedPose2D()
	self.xPos = 0
	self.yPos = 0
	self.zPos = 0
	self.tow = 0
	self.course = 0.0

class IvyCalibrationNode:
    def __init__(self):
        self.myKillLog = kill_log.KillLog()
    
    def IvyInitStart(self):
        """ Initializes the Ivy Server and ROS Subscriber

        Should only be called once per session.
        """
	

        try:
            IvyInit('Calibration Node', '', 0)
        except AssertionError:
            print('Assertion Error in IvyInit(!= none), is there a server already running? Exiting')
            IvyStop()
            raise SystemExit()
        IvyStart()
        try:
            self.initRosSub()
        except rospy.exceptions.ROSInitException as e:
            print('\nInitialization failed due to ROS error, exiting...')
            self.IvyInitStop()
        time.sleep(1)
        print('Ivy Calibration Node ready!')
	
        # initial value for oldTime
	global oldDataBlue
	oldDataBlue = OldData()
	global oldDataCyan
	oldDataCyan = OldData()	

	#global oldX
	#oldX = 0
	#global oldY
	#oldY = 0
	#global oldZ
	#oldZ = 0
	#global tow
	#tow = 0
        #global oldTime 
	#oldTime = rospy.get_rostime()
	#global oldData
	#global course
	#course = 0.0

        rospy.spin()
        print("after ctrl + c")
        self.IvyInitStop()


    def IvyInitStop(self):
        """Stops the Ivy Server.
        """
        time.sleep(1)
        IvyStop()


    def handlePos(self, data, ID):
        """ Callback for the ROS subscriber.


        """
	global oldDataBlue
	global oldDataCyan 
	global wall_hit_count      
	global wall_was_hit 
	#global oldTime
	#global oldData
	#global oldX
	#global oldY
	#global oldZ
	#global tow
	#global course

	oldData = OldData()	
	
	if (ID == 3):
		oldData = oldDataBlue
	elif (ID == 7):
		oldData = oldDataCyan	

	# offsets for camera positions
	offsetX =  data.x
	offsetY =  data.y
	offsetZ =  0

	# getting time difference between now and last run
	now = data.header.stamp
	timediff = 0
	if( (now.secs - oldData.time.secs) == 0):
		timediff = now.nsecs - oldData.time.nsecs
	else:
		timediff = 1000000000 + now.nsecs - oldData.time.nsecs
	oldData.time = now
	timediff = timediff / 1000000000.0
	
	#rospy.loginfo("timediff float %f", timediff)

	# geting difference for X pos and Y pos and Z pos and calculate speed
	
	ecef_xd= (offsetX - oldData.xPos) / timediff	
	ecef_yd= (offsetY - oldData.yPos) / timediff	
	ecef_zd= (offsetZ - oldData.zPos) / timediff	
	
	#rospy.loginfo("Speeds %f, %f, %f", ecef_xd, ecef_yd, ecef_zd)

	oldData.xPos = offsetX
	oldData.yPos = offsetY
	oldData.zPos = offsetZ

	# for TOW
	if (oldData.tow == 0):
		oldData.tow = now.secs * 1000 + int(now.nsecs / 1000000)
		oldData.tow = oldData.tow % weekInMilliseconds

	if (timediff < 1.0):
		oldData.tow += int(timediff * 1000)
		oldData.tow = oldData.tow % weekInMilliseconds

	rospy.loginfo("ID %d !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", ID)
	
	previousCourse = oldData.course	

	try:
		distX = data.x - oldData.xPos
		distY = data.y - oldData.yPos
		dist = math.sqrt(distX*distX + distY*distY)		
	
		oldData.course = math.acos(distX / dist)

		if(distY < 0):
			oldData.course = 2*math.pi - oldData.course
	except:
		oldData.data = data
		rospy.loginfo("error olddata")
		oldData.course = previousCourse/10000000.0
	
	rospy.loginfo("course %f", oldData.course)
	# course in rad*1e7, [0, 2*Pi]*1e7 (CW/north)

	oldData.course = int(oldData.course * 10000000)

	earthRadius = 636485000
	ecef_magnitude = math.sqrt((384205200 + offsetX)*(384205200 + offsetX) + (79184900 + offsetY)*(79184900 + offsetY) + (501233200 + offsetZ)*(501233200 + offsetZ))
	hmsl = (ecef_magnitude - earthRadius) * 10 # in mm
        #test value
	hmsl = 1000;

	if (ID == 3):
        	#AC_ID, numsv, ecef_x, ecef_y, ecef_z, 					     lat, lon, alt, hmsl, ecef_xd, ecef_yd, ecef_zd, tow, course
		self.IvySendRemoteGPS(ID,     6,     384205200 + data.x , 79184900 + data.y, 501233200 + 60,      gps_lat,   gps_lon,   0,   hmsl,  ecef_xd, ecef_yd, ecef_zd, oldData.tow, oldData.course)

	rospy.loginfo("ID :%d   gps-x %s  gps-y %s", ID, 384205200 + data.x, 79184900 + data.y)

        # loop for sending GPS for swarmbehaviour to all other copters
	#IvySendGPSBroadcast(AC_ID, copter_id, gps_x, gps_y, gps_z, gps_xd, gps_yd, gps_zd)
	#if (ID == 3):	
	#	self.IvySendGPSBroadcast(7, 3, 384205200 + data.x, 79184900 + data.y, 501233200 + 60, ecef_xd, ecef_yd, ecef_zd)
	#el
	if (ID == 7):
		self.IvySendGPSBroadcast(3, 7, 384205200 + data.x, 79184900 + data.y, 501233200 + 60, ecef_xd, ecef_yd, ecef_zd)

	#send camera heading in degree
	self.IvySendCameraTheta(ID, 0, data.theta + 185)

	oldData.data = data

	# position for virtual copter // (To, From, Pos_x, Pos_x, Pos_x,, 0, 0, 0) 
	self.IvySendGPSBroadcast(3, 5, virt_pos_x, virt_pos_y, virt_pos_z, 0, 0, 0)
	
	# sending evaluation data
	# dist to virt copter
	dist_x = virt_pos_x - (384205200 + data.x)
	dist_y = virt_pos_y - (79184900 + data.y)
	dist_to_virtual = math.sqrt( dist_x*dist_x + dist_y*dist_y)

	# check wall hit
	if ( ((384205200 + data.x) < (wall_top_x + safe_dist)) | ((384205200 + data.x) > (wall_bottom_x - safe_dist))
		| ((79184900 + data.y) < (wall_left_y + safe_dist)) | ((79184900 + data.y) > (wall_right_y - safe_dist))):
		if(wall_was_hit == False):
			wall_hit_count =  wall_hit_count + 1
		wall_was_hit = True
	else:
		wall_was_hit = False
	
	
	IvySendMsg('3 SWARM_EVALUATION %f %d' % (dist_to_virtual, wall_hit_count))

    def initRosSub(self):
        """ Initializes the ROS subscriber.

        Is automatically called during the Ivy initialization process
        in IvyInitStart().
        """
        try:
            rospy.init_node('poseListener', anonymous=False)
        except KeyboardInterrupt:
            print('\nROS initialization canceled by user')
        except rospy.exceptions.ROSInitException as e:
            print('\nError Initializing ROS:')
            print(str(e))
            raise
	
        #rospy.Subscriber("copters/0/pose", Pose2D, self.handlePos)
	rospy.Subscriber("/copter/blue", TaggedPose2D, self.handlePos, 3)
	#rospy.Subscriber("/copter/magenta", TaggedPose2D, self.handlePos, 3)
	#rospy.Subscriber("/copter/cyan", TaggedPose2D, self.handlePos, 7)
	#rospy.Subscriber("/copter/yellow", TaggedPose2D, self.handlePos, 7)
	#rospy.Subscriber("/copter/white", TaggedPose2D, self.handlePos, 7)


    def IvyGetPos(self):
        """Simply returns the position grabbed via ROS to the caller

        """
        try:
            return copterPos
        except NameError as e:
            print("CopterPos not yed defined! (No data from ROS?):")
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

    def IvySendCameraTheta(self, AC_ID, dummy, theta):
        IvySendMsg('dl CAMERA_THETA %d %d %f' %
                    (AC_ID, dummy, theta) )

    def IvySendRemoteGPS(self, AC_ID, numsv, ecef_x, ecef_y, ecef_z, lat, lon, alt, hmsl, ecef_xd, ecef_yd, ecef_zd, tow, course):
	#rospy.loginfo("gps-x %s  gps-y %s", (ecef_x), (ecef_y))
        IvySendMsg('dl REMOTE_GPS %d %d %d %d %d %d %d %d %d %d %d %d %d %d' %
                    (AC_ID, numsv, int(ecef_x), int(ecef_y), int(ecef_z), lat, lon, alt, hmsl, int(ecef_xd), int(ecef_yd), int(ecef_zd), tow, course
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


if __name__ == '__main__':
    comm = IvyCalibrationNode()
    comm.IvyInitStart()
