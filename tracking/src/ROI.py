"""
    Region-of-Interest(ROI) Implementation for tracking
"""

import numpy as np
from enum import Enum

class RoIParams(Enum):
    """
        Configuration parameters for Region-of-Interest(ROI) objects
    """
    initial_size = 1
    size_increment = 2

class RoI(object):
    """
        Region-of-Interest(ROI) tracking an object in an image
    """
    def __init__(self, roi_params):
        self.position = None
        self.speed = None
        self.estimation = None
        self.params = roi_params
        self.size = roi_params[RoIParams.initial_size]

    def update(self, pose):
        """
            Update pose of RoI based on current pose of object. It will
            extrapolate the future pose and size of the RoI according to
            the configured parameters
            :param pose: current pose(x, y, theta) of tracked object
        """
        if not self.position == None:
            self.speed = pose - self.position
            self.estimation = pose + self.speed
        self.position = pose


    def crop(self, img, increased=False):
        if self.speed == None or self.estimation == None:
            return img
        print self.estimation
        if increased:
            self.size += self.params[RoIParams.size_increment]
        else:
            self.size = self.params[RoIParams.initial_size]
        start = self.estimation.astype("int")-np.ones(2)*int(self.size/2)
        end = start + np.ones(2)*self.size
        start = np.minimum(np.maximum(start, np.zeros(2)), img.shape[0:2])
        end = np.minimum(end, img.shape[0:2])
        res = img[start[0]:end[0], start[1]:end[1], :]
	return res

    def map_coords(self, points):
        if self.speed == None or self.estimation == None:
            return points
        coord_trans = lambda pt: pt+self.estimation+np.ones(2)*self.size/2
        return map(coord_trans, points)

    def __repr__(self):
        return "ROI at (%.2f, %.2f): %s"%(self.x(), self.y(), self.size)
