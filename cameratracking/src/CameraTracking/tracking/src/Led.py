"""
    Led abstraction enabling configurable color detection and classification
"""

from enum import Enum
import numpy as np
import colorsys

class LedParams(Enum):
    """
        Led color detection configuration parameters
    """
    colors = 1
    size = 2
    hs_values = 3


# general LED class
class Led(object):
    """
    An LED object has coordinates and a color attribute.
    """
    def __init__(self, point, img, led_params):
        """
        Given a raw image with a defined point, pick color.

        :param img: Raw CV2 BGR image
        :param center_x: x-coordinate
        :param center_y: y-coordinate
        :return: A color from the color enumerator
        """
        self.params = led_params
        self.point = point
        size = int(led_params[LedParams.size]/2)
	start = point - size
	end = start + 2* size +1
        color_region = img[start[1]:end[1], start[0]:end[0], :]
        avg_blu = np.mean(color_region[:, :, 0])/255
        avg_gre = np.mean(color_region[:, :, 1])/255
        avg_red = np.mean(color_region[:, :, 2])/255
	self.rgb_color = np.array([avg_red, avg_gre, avg_blu])
        self.raw_color = np.array(colorsys.rgb_to_hsv(avg_red, avg_gre, avg_blu))

        self.color = None

        hsv = np.array(led_params[LedParams.hs_values].values())
        max_h = np.max(hsv[:, 0, 0])
        min_h = np.min(hsv[:, 0, 0])
        for color in led_params[LedParams.colors]:
            reference = np.array(led_params[LedParams.hs_values][color][0])
            offset = np.array(led_params[LedParams.hs_values][color][1])
            comp = self.raw_color[0:2]-reference
            if reference[0] <= min_h and self.raw_color[0] > max_h:
                comp[0] = 1-comp[0]
            if (np.absolute(comp) <= offset).all():
                self.color = color
                break
	if not self.color:
		self.color="unknown"

    def x(self):
        """
            Get x coordinate of led
            :return: x coordinate
        """
        return self.point[0]

    def y(self):
        """
            Get y coordinate of led
            :return: y coordinate
        """
        return self.point[1]

    def rgb(self):
	if self.color=="unknown":
		return np.array([0, 0, 0])
	else:
        	return np.array(colorsys.hsv_to_rgb(*self.params[LedParams.hs_values][self.color][0], v=1))*255

    def transform(self, homography):
        temp = np.zeros((3, ))
        temp[0:2] = self.point
        temp[2] = 1
        self.point = homography.dot(temp)
        self.point = self.point / self.point[2]
        self.point = self.point[0:2]


    def __str__(self):
        return 'LED (%.2f, %.2f): %s (hsv: %.2f, %.2f, %.2f) (rgb: %.2f, %.2f, %.2f)' \
                %(self.x(), self.y(), self.color, self.raw_color[0], self.raw_color[1], self.raw_color[2], self.rgb_color[0], self.rgb_color[1], self.rgb_color[2])

    def __repr__(self):
        return str(self)
