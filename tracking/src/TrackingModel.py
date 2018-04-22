import rospy
from tracking.msg import TaggedPose2D
import numpy as np
from ROI import RoI
from cv2 import SimpleBlobDetector_Params

class TrackingObject(TaggedPose2D, object):
    def __init__(self, roi_params, func):
        TaggedPose2D.__init__(self)
        self.roi = RoI(roi_params)
        self.comp_center = func[0]
        self.comp_theta = func[1]
        self.comp_id = func[2]
        self.comp_quality = func[3]

    def position(self):
        return np.array([self.x, self.y])

    def pose(self):
        return  np.array([self.x, self.y, self.theta])

    def update(self, leds, timestamp=None):
        if not timestamp:
            timestamp = rospy.get_rostime()
        self.header.stamp = timestamp
        result = self.comp_center(leds) and \
                 self.comp_theta(leds) and \
                 self.comp_id(leds) and \
                 self.comp_quality(leds)
        self.roi.update(self.position())
        return result

    def __repr__(self):
        return "%s(%s, %s, %s): %s"%(self.id, self.x, self.y, self.theta, self.quality)

    def __str__(self):
        return repr(self)

class  TrackingModel(object):
    def __init__(self, name="Generic Tracking Model"):
        self.led_params = {}
        self.roi_params = {}
        self.cluster_params = {}
        self.blob_params = SimpleBlobDetector_Params()
        self.name = name

    def __repr__(self):
        return self.name+":\nled params: %s\nroi_params: %s\ncluster params: %s" \
               %(self.led_params, self.roi_params, self.cluster_params)

    def __str__(self):
        return  repr(self)

    def create(self):
        return None
