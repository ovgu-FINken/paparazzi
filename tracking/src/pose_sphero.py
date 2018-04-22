#!/usr/bin/env python2

"""
The python script for tracking moving copters, while publishing their pose info.
Each copter must be equipped with five LEDs. Two front green LEDs, two rear
red LEDs, and a center LED for the ID.
"""

import os
import numpy as np


import rospy
from Tracker import Tracker
from tracking.msg import TaggedPose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from SpheroModel import SpheroModel

tracker = None
pubs = {}
pose = TaggedPose2D()

def resize_pubs(name):
    if not name in pubs:
        pubs[name]=rospy.Publisher('/sphero/'+name, TaggedPose2D, queue_size=1)

def callback(msg):
    try:
        img_original = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
        tracker.update(img_original, msg.header.stamp)
        for obj in tracker.objects:
            resize_pubs(obj.id)
            pubs[obj.id].publish(obj)
    except CvBridgeError as error:
        print(error)

if __name__ == '__main__':
    rospy.init_node('pose_estimate')
    homography = None
    try:
        homography = rospy.get_param("~homography_file")
        filename = rospy.get_param('~homography_file')
        if os.path.isfile(filename):
            homography = np.load(filename)
        else:
            rospy.logerr("Homography_file: %s not found."
                         "Output will be in camera coordinates", filename)
    except KeyError:
        rospy.logerr("No homography_file set. Output will be in camera coordinates")
    tracker = Tracker(SpheroModel(), homography)
    subscriber = rospy.Subscriber('/camera/0', Image, callback, queue_size=1)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
