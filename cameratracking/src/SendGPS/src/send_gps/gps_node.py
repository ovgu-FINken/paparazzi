#!/usr/bin/env python

import time
import logging
import json
from collections import defaultdict
import math
import rospy
from tracking.msg import TaggedPose2D
from std_api import IvyInit, IvySendMsg, IvyStop, IvyStart


# create logger with 'spam_application'
LOGGER = logging.getLogger('send_gps')
LOGGER.setLevel(logging.DEBUG)

ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
LOGGER.addHandler(ch)

# global values
DEG2RAD = math.pi / 180
RAD2DEG = 180 / math.pi
FIN_LATITUDE = 52.138821 * DEG2RAD  # Latitude of FIN
FIN_LONGITUDE = 11.645634 * DEG2RAD # Longitude of FIN
GPS_LATITUDE = FIN_LATITUDE * 10000000
GPS_LONGITUDE = FIN_LONGITUDE * 10000000
WEEK_MILLISECONDS = 604800000
EARTH_RADIUS = 636485000


class CopterData:

    def __init__(self):
        self.time = rospy.get_rostime()
        self.data = TaggedPose2D()
        self.x = 0
        self.y = 0
        self.z = 0
        self.tow = 0
        self.course = 0.0


class GPSNode:

    def __init__(self, config_path=None):

        self.copter_data_dict = defaultdict(CopterData)

        self.rostopic_copterid_dict = {
            '/copter/cyan': 3,
            '/copter/blue': 3,
            '/copter/magenta': 3,
            '/copter/yellow': 3,
            '/copter/white': 3,
        }

        if config_path:
            LOGGER.info("Config found! Loading from %s" % config_path)
            with open(config_path) as json_file:
                self.rostopic_copterid_dict = json.load(json_file)

                # convert string ids to int
                for key in self.rostopic_copterid_dict:
                    self.rostopic_copterid_dict[key] = int(self.rostopic_copterid_dict[key])

    def run(self):
        try:
            IvyInit('Calibration Node', '', 0)
        except AssertionError:
            LOGGER.error('Assertion Error in IvyInit(!= none), is there a server already running? Exiting')
            IvyStop()
            raise SystemExit()

        IvyStart()
        try:
            self.initRosSub()
        except rospy.exceptions.ROSInitException:
            LOGGER.error('Initialization failed due to ROS error, exiting...')
            self.stop()

        time.sleep(1)
        LOGGER.info('Ivy Calibration Node has started!')

        rospy.spin()
        LOGGER.info("Ros-Spin exited")
        self.stop()

    def stop(self):
        time.sleep(1)
        IvyStop()

    def handlePos(self, current_copter_data, copter_id):
        """ Callback for the ROS subscriber."""
        LOGGER.info("Received Position Data from ID %s" % copter_id)

        previous_copter_data = self.copter_data_dict[copter_id]

        # getting time difference between now and last run
        now = current_copter_data.header.stamp
        LOGGER.debug("Received Timestamp %i %i" % (now.secs, now.nsecs))

        time_diff = (now-previous_copter_data.time).to_sec()
        LOGGER.info('TimeDiff between last update: %s' % time_diff)

        # offsets for camera positions
        offsetX = current_copter_data.x
        offsetY = current_copter_data.y
        offsetZ = 0

        # geting difference for X pos and Y pos and Z pos and calculate speed
        ecef_xd = (offsetX - previous_copter_data.x) / time_diff
        ecef_yd = (offsetY - previous_copter_data.y) / time_diff
        ecef_zd = (offsetZ - previous_copter_data.z) / time_diff

        previous_copter_data.time = now
        previous_copter_data.x = offsetX
        previous_copter_data.y = offsetY
        previous_copter_data.z = offsetZ

        # for TOW
        if previous_copter_data.tow == 0:
            previous_copter_data.tow = now.secs * 1000 + int(now.nsecs / 1000000)
            previous_copter_data.tow = previous_copter_data.tow % WEEK_MILLISECONDS

        if time_diff < 1.0:
            previous_copter_data.tow += int(time_diff * 1000)
            previous_copter_data.tow = previous_copter_data.tow % WEEK_MILLISECONDS

        previousCourse = previous_copter_data.course
        try:
            distX = current_copter_data.x - previous_copter_data.x
            distY = current_copter_data.y - previous_copter_data.y
            dist = math.sqrt(distX**2 + distY**2)

            previous_copter_data.course = math.acos(distX / dist)

            if distY < 0:
                previous_copter_data.course = 2*math.pi - previous_copter_data.course
        except ZeroDivisionError as e:
            previous_copter_data.course = previousCourse/10000000.0 # TODO WHY?

        LOGGER.info("Course: %s" % previous_copter_data.course)
        rospy.loginfo("course %f", previous_copter_data.course)

        # course in rad*1e7, [0, 2*Pi]*1e7 (CW/north)
        previous_copter_data.course = int(previous_copter_data.course * 10000000)

        CONST_1 = 384205200
        CONST_2 = 79184900
        CONST_3 = 501233200

        ecef_magnitude = math.sqrt(
            (CONST_1 + offsetX)**2 +
            (CONST_2 + offsetY)**2 +
            (CONST_3 + offsetZ)**2
        )

        hmsl = (ecef_magnitude - EARTH_RADIUS) * 10  # in mm

        self.IvySendRemoteGPS(
            copter_id,
            6,
            CONST_1 + current_copter_data.x,
            CONST_2 + current_copter_data.y,
            CONST_3 + 60,
            GPS_LATITUDE,
            GPS_LONGITUDE,
            0,
            hmsl,
            ecef_xd,
            ecef_yd,
            ecef_zd,
            previous_copter_data.tow,
            previous_copter_data.course
        )

        LOGGER.info("ID :%d   gps-x %s  gps-y %s", copter_id, CONST_1 + current_copter_data.x, CONST_2 + current_copter_data.y)

        # send camera heading in degree
        self.IvySendCameraTheta(copter_id, 0, current_copter_data.theta + 185)

        # update old data
        previous_copter_data.data = current_copter_data

    def initRosSub(self):

        try:
            rospy.init_node('poseListener', anonymous=False)
        except rospy.exceptions.ROSInitException as e:
            LOGGER.exception("Error Initializing ROS")
            raise e

        for ros_topic, copter_id in self.rostopic_copterid_dict.items():
            LOGGER.info("Subscribing Topic %s for CopterID %s" % (ros_topic, copter_id))
            rospy.Subscriber(name=ros_topic, data_class=TaggedPose2D, callback=self.handlePos, callback_args=copter_id)

    def IvySendCameraTheta(self, AC_ID, dummy, theta):
        IvySendMsg('dl CAMERA_THETA %d %d %f' %
                    (AC_ID, dummy, theta) )

    def IvySendRemoteGPS(self, AC_ID, numsv, ecef_x, ecef_y, ecef_z, lat, lon, alt, hmsl, ecef_xd, ecef_yd, ecef_zd, tow, course):

        IvySendMsg('dl REMOTE_GPS %d %d %d %d %d %d %d %d %d %d %d %d %d %d' %
                    (AC_ID, numsv, int(ecef_x), int(ecef_y), int(ecef_z), lat, lon, alt, hmsl, int(ecef_xd), int(ecef_yd), int(ecef_zd), tow, course
                    ))


