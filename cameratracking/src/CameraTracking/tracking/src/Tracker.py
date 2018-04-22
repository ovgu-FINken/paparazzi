

import cv2
import rospy
from Cluster import compute_clusters, ClusterParams
from Led import Led
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
"""
Blob detector parameters. We are ignoring shapes, and only look for something
bigger than the minArea pixels.
"""

# enumerators for color values


class Tracker(object):
    """
    Detector tracking multiple objects in camera images and
    output recognised 2D-Point and orienation
    """
    # get an image, and return blob keypoints
    def get_keypoints(self, img):
        """
        Given a raw image, get the keypoints from blob detector.

        :param img: Raw CV2 BGR image
        :return: An array of keypoints
        """
        blurred = cv2.GaussianBlur(img, (5, 5), sigmaX=0, sigmaY=0)
        self.debug(blurred, "blur", 'bgr8')
        #norm = np.zeros(img.shape, dtype="uint8")
        #cv2.normalize(blurred, norm, 0, 255, cv2.NORM_MINMAX)
        #self.debug(norm, "norm", 'bgr8')
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        self.debug(gray, "gray", 'mono8')
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, -10)
        #_, thresh = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)
        #_, thresh = cv2.threshold(gray,   0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        self.debug(thresh, "thresh", 'mono8')
        keypoints = self.detector.detect(thresh)
        return map(lambda kpt: np.array(kpt.pt).astype(int), keypoints)

    def debug(self, img, name=None, encoding='rgb8'):
        if name == None:
            name = ""
        if not name in self.debug_pubs:
            self.debug_pubs[name] = rospy.Publisher(rospy.get_name()+"/debug/"+name, Image, queue_size=1)
        try:
            msg = CvBridge().cv2_to_imgmsg(img.astype("uint8"), encoding)
            self.debug_pubs[name].publish(msg)
        except CvBridgeError as error:
            print(error)

    def update(self, img, time):
        """
        Callback function for the image messages

        :param msg: ROS message containing the image
        """

        to_led = lambda point: Led(point, img, self.model.led_params)

        # update self.clusters once a while
        if True or not self.objects or time - self.old_time > self.model.cluster_params[ClusterParams.timeout]:
            self.objects = []
            keypoints = self.get_keypoints(img)
            print("found %s keypoints"%len(keypoints))
            clusters = compute_clusters(keypoints, self.model.cluster_params)
            debug_img = np.zeros(img.shape)
            for cluster in clusters:
                obj = self.model.create()
                leds = map(to_led, cluster)
                for led in leds:
                    if self.homography is not None:
                        led.transform(self.homography)
                    for i in range(-1, 2):
                        for j in range(-1, 2):
                            debug_img[led.y()+i, led.x()+j, :] = led.rgb()
                    print led
                if obj.update(leds, time):
                    self.objects.append(obj)
            self.debug(debug_img)
        else:
            debug_img = np.zeros(img.shape)
            for obj in self.objects:
                roi_img = obj.roi.crop(img)
                keypoints = self.get_keypoints(roi_img)
                obj.roi.map_coords(keypoints)
                leds = map(to_led, keypoints)
                if not obj.update(ledsi, time):
                    roi_img = obj.roi.crop(img, True)
                    keypoints = self.get_keypoints(roi_img)
                    obj.roi.map_coords(keypoints)
                    leds = map(to_led, keypoints)
                    obj.update(leds, time)
                for led in leds:
                    debug_img[led.y(), led.x(), :] = led.rgb()
            self.debug(debug_img)
        self.old_time = time

    def __init__(self, model, homography=None):
        self.model = model
        self.homography = homography
        try:
            self.detector = cv2.SimpleBlobDetector_create(model.blob_params)
        except AttributeError:
            self.detector = cv2.SimpleBlobDetector(model.blob_params)
        self.objects = []
        self.old_time = rospy.Time(0)
        self.debug_pubs = {}
