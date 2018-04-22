"""
    FINken-III-Max Tracking Model implementation
"""

from TrackingModel import TrackingObject, TrackingModel
from ROI import RoIParams
from Cluster import ClusterParams
from Led import LedParams
from tracking.msg import TaggedPose2D
import numpy as np
import rospy
import math


class Sphero(TrackingObject):

    def __compute_theta(self,leds):

        self.theta=0
        return True  
   
    def __compute_center(self,leds):
        led=[led for led in leds if led.color!="unknown"]
        if len(led)>0:
         diag=[0,0]
         diag=led[0].point
         self.x=diag[0]
         self.y=diag[1]
         return True

        return False

    def __compute_id(self,leds):
        id_leds = [led for led in leds if led.color!="unknown"]
        if len(id_leds) == 0:
            rospy.logwarn("No id led found for sphero at (%s, %s)", self.x, self.y)
            self.id = 0
            return False
        else:
           self.id=str(id_leds[0].color)

           return True
        #self.id = str(id_leds[0].color)
        return False

    def __compute_quality(self,leds):
    
        self.quality=10
        return True

    def __init__(self, roi_params):
        TrackingObject.__init__(self, roi_params, (self.__compute_center, self.__compute_theta, self.__compute_id, self.__compute_quality))



class SpheroModel(TrackingModel):

    def __init__(self):
        super(SpheroModel, self).__init__("Sphero tracking model")
        self.name='Sphero/'
        self.cluster_params[ClusterParams.max_dist] = 10
        self.cluster_params[ClusterParams.min_leds] = 1
        self.cluster_params[ClusterParams.max_leds] = 1
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
        self.led_params[LedParams.colors] = ["blue", "green", "red","yellow","cyan","magenta"]
        self.led_params[LedParams.hs_values] = {"blue":((0.66, 0.8), (0.15, 0.2)),
                                                "green":((0.33, 0.7), (0.15, 0.3)),
                                                "red":((0.0, 0.5), (0.15, 0.5)),
                                                "yellow":((0.16,0.7), (0.15, 0.3)),
                                                "cyan":((0.5,0.8), (0.15,0.2)),
                                                "magenta":((0.83,0.7), (0.15,0.3))}

    def create(self):
        return Sphero(self.roi_params)

