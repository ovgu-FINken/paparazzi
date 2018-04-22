"""
    FINken-III-Max Tracking Model implementation
"""

from enum import Enum
from TrackingModel import TrackingObject, TrackingModel
from ROI import RoIParams
from Cluster import ClusterParams
from Led import LedParams
import numpy as np
import rospy
import math

class Car(TrackingObject):
    """
        FINken-III-Max copter tracking object
    """

    def __compute_theta(self, leds):
        """
        Given an array of LEDs of a copter, calculate its angle.

        :param leds: Array of objects of the LED class
        :return: Angle in degrees, None is not calculated
        """
        front_leds = [led for led in leds if led.color == "cyan"]
        back_leds = [led for led in leds if led.color == "red"]

        if len(front_leds) + len(back_leds) < 3 or len(front_leds) == 0 or len(back_leds) == 0:
            #rospy.logwarn("Not enough leds for orientation computation")
            return False

        #do we have 2 front leds?
        if len(front_leds) == 2 and len(back_leds) > 0:
            #get a vector that connects them
            front_vec = front_leds[1].point-front_leds[0].point
            #rotate it by 90 degress
            front_vec_90 = np.array([-front_vec[1], front_vec[0]])
            #get a vector that connects a front to a back
            side_vec = back_leds[0].point-front_leds[0].point
            #get the inner product of the two, if its negative we have a pose
            #otherwise rotate fvect to -90
            if np.inner(front_vec_90, side_vec) > 0:
                front_vec_90 = np.array([front_vec[1], -front_vec[0]])
            self.theta = np.arctan2(front_vec_90[1], front_vec_90[0])*180/np.pi
            #return the angle
        #or we have 2 back leds?
        if len(back_leds) == 2 and len(front_leds) > 0:
            back_vec = back_leds[1].point-back_leds[0].point
            back_vec_90 = np.array([-back_vec[1], back_vec[0]])
            side_vec = front_leds[0].point-back_leds[0].point
            if np.inner(np.array(back_vec_90), np.array(side_vec)) < 0:
                back_vec_90 = np.array([back_vec[1], -back_vec[0]])
            self.theta = np.arctan2(back_vec_90[1], back_vec_90[0])*180/np.pi

        if self.theta < 0:
            self.theta += 360
        return True

    def __compute_center(self, leds):
        """
        Given an array of of LEDs of a copter, find its center.

        :param leds: An array of objects of the LED class
        :return: A point of the calculated angle, None if not calculated.
        """
        front_leds = [led for led in leds if led.color == "cyan"]
        back_leds = [led for led in leds if led.color == "red"]

	print "got %r" % leds

        if len(front_leds) + len(back_leds) < 3 or len(front_leds) == 0 or len(back_leds) == 0:
            #rospy.logwarn("Not enough leds for center computation")
            return False
        # reference LED will be the one which is alone in its group
        diag = [0, 0]
        if len(front_leds) == 1:
            maxd = 0
            # compare with both LEDs and pick one with the biggest distance
            for i in range(2):
                dist = front_leds[0].point-back_leds[i].point
                dist = np.linalg.norm(dist)
                if maxd < dist:
                    maxd = dist
                    diag = front_leds[0].point+back_leds[i].point
        else:
            maxd = 0
            for i in range(2):
                dist = back_leds[0].point-front_leds[i].point
                dist = np.linalg.norm(dist)
                if maxd < dist:
                    maxd = dist
                    diag = front_leds[i].point+back_leds[0].point
        self.x = (diag/2)[0]
        self.y = (diag/2)[1]
        return True

    def __compute_id(self, leds):
        self.id = 0
        return True

    def __compute_quality(self, leds):
        id_leds = [led for led in leds if led.color != "green" and led.color != "red"]
        front_leds = [led for led in leds if led.color == "green"]
        back_leds = [led for led in leds if led.color == "red"]
        self.quality = math.pow(2.0, -(abs(len(id_leds)-1)+abs(len(front_leds)-2)+abs(len(back_leds)-2)))/2
        return True

    def __init__(self, roi_params):
        TrackingObject.__init__(self, roi_params, (self.__compute_center, self.__compute_theta, self.__compute_id, self.__compute_quality))

class CarModel(TrackingModel):
    """
        Ottocar Ada-III Tracking Model containing parameter sets and create function to spawn Cars
    """


    def __init__(self):
        super(CarModel, self).__init__("Ottocar Ada-III Model")
        self.cluster_params[ClusterParams.max_dist] = 100
        self.cluster_params[ClusterParams.min_leds] = 3
        self.cluster_params[ClusterParams.max_leds] = 5
        self.cluster_params[ClusterParams.timeout] = rospy.rostime.Duration.from_sec(1)
        self.roi_params[RoIParams.initial_size] = 100
        self.roi_params[RoIParams.size_increment] = 100
        self.blob_params.filterByInertia = False
        self.blob_params.filterByConvexity = False
        self.blob_params.filterByColor = False
        self.blob_params.filterByCircularity = False
        self.blob_params.filterByArea = True
        self.blob_params.minArea = 1
        #blue: 0.66, green: 0.33, red: 0, yellow: 0.16, cyan: 0.5, magenta: 0.83
        self.led_params[LedParams.size] = 5
        self.led_params[LedParams.colors] = ["cyan", "red"]
        self.led_params[LedParams.hs_values] = {"cyan":((0.5, 0.6), (0.2, 0.4)),
                                                "red":((0.0, 0.6), (0.2, 0.4)),
						"green":((0.33, 0.6), (0.15, 0.4)),
                                                "white":((0.5, 0.2), (0.5, 0.1))}

    def create(self):
        return Car(self.roi_params)
