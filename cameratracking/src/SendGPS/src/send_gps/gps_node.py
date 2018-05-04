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
FIN_LATITUDE = 52.138821 * DEG2RAD   # Latitude of FIN
FIN_LONGITUDE = 11.645634 * DEG2RAD  # Longitude of FIN
GPS_LATITUDE = FIN_LATITUDE * 10000000
GPS_LONGITUDE = FIN_LONGITUDE * 10000000
# ECEF Coordinates of FIN (umgerechnete LAT/Longitude) in cm
FIN_ECEF_X = 384205200
FIN_ECEF_Y = 79184900
FIN_ECEF_Z = 501233200
WEEK_MILLISECONDS = 604800000
EARTH_RADIUS = 636485000


class CopterData:

    def __init__(self):
        self.time = rospy.get_rostime()
        self.x_cm = 0
        self.y_cm = 0
        self.z_cm = 0
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
        LOGGER.info("Received Position Data from ID %s: x=%s, y=%s, z=%s" % (copter_id, current_copter_data.x, current_copter_data.y, 0))

        previous_copter_data = self.copter_data_dict[copter_id]

        # getting time difference between now and last run
        now = current_copter_data.header.stamp
        LOGGER.debug("Received Timestamp %i %i" % (now.secs, now.nsecs))

        time_delta_sec = (now-previous_copter_data.time).to_sec()
        LOGGER.info('TimeDiff between last update: %s' % time_delta_sec)

        # getting difference for X pos and Y pos and Z pos
        delta_x_cm = current_copter_data.x - previous_copter_data.x_cm
        delta_y_cm = current_copter_data.y - previous_copter_data.y_cm
        delta_z_cm = 0 - previous_copter_data.z_cm
        LOGGER.info("Position-Delta (cm): x=%s, y=%s, z=%s" % (delta_x_cm, delta_y_cm, delta_z_cm))

        # calculate speed
        xd_cm_per_s = delta_x_cm / time_delta_sec
        yd_cm_per_s = delta_y_cm / time_delta_sec
        zd_cm_per_s = delta_z_cm / time_delta_sec
        LOGGER.info("Calculated Speed: xd=%s, yd=%s, zd=%s" % (xd_cm_per_s, yd_cm_per_s, zd_cm_per_s))

        try:
            # TODO what does the course of the copter mean?
            # atan2 von delta y und delta x

            dist = math.sqrt(delta_x_cm**2 + delta_y_cm**2)
            previous_copter_data.course = int(math.acos(delta_x_cm / dist) * 10000000)  # Throws error if dist==0

            if delta_y_cm < 0:
                previous_copter_data.course = 2*math.pi - previous_copter_data.course

            LOGGER.info("Course: %s" % previous_copter_data.course)
            # Unterscheide ich 
        except ZeroDivisionError:
            pass

        # course in rad*1e7, [0, 2*Pi]*1e7 (CW/north)
        ecef_magnitude = math.sqrt(
            (FIN_ECEF_X + current_copter_data.x) ** 2 +
            (FIN_ECEF_Y + current_copter_data.y) ** 2 +
            0
        )
        hmsl = (ecef_magnitude - EARTH_RADIUS) * 10  # in mm

        # for Time of Week
        if previous_copter_data.tow == 0:
            previous_copter_data.tow = now.secs * 1000 + int(now.nsecs / 1000000)
            previous_copter_data.tow = previous_copter_data.tow % WEEK_MILLISECONDS

        if time_delta_sec < 1.0:
            previous_copter_data.tow += int(time_delta_sec * 1000)
            previous_copter_data.tow = previous_copter_data.tow % WEEK_MILLISECONDS

        # send current data to copter
        self.IvySendRemoteGPS(
            copter_id,
            6,
            FIN_ECEF_X + current_copter_data.x,
            FIN_ECEF_Y + current_copter_data.y,
            0,
            GPS_LATITUDE,
            GPS_LONGITUDE,
            0,
            hmsl,
            xd_cm_per_s,
            yd_cm_per_s,
            zd_cm_per_s,
            previous_copter_data.tow,
            previous_copter_data.course
        )

        LOGGER.info("ID :%d   gps-x %s  gps-y %s", copter_id, FIN_ECEF_X + current_copter_data.x, FIN_ECEF_Y + current_copter_data.y)

        # send camera heading in degree, for the virtual Magnetometer
        self.IvySendCameraTheta(copter_id, 0, current_copter_data.theta + 185)

        # update copter data
        previous_copter_data.time = now
        previous_copter_data.x_cm = current_copter_data.x
        previous_copter_data.y_cm = current_copter_data.y
        previous_copter_data.z_cm = 0

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


