import unittest
import colorsys

PKG = 'tracking'

import rospkg
import sys
rospkg = rospkg.RosPack()
sys.path = [rospkg.get_path(PKG)+"/src"]+sys.path

from enum import Enum
from Led import LedParams, Led
import numpy  as np

class Color(Enum):

    __order__ = "white green red blue yellow  magenta cyan unknown"
    green = 1
    red = 2
    blue = 3
    yellow = 4
    magenta = 5
    cyan = 6
    white = 7
    unknown = 20

class TestLedColorDetect(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.params = {LedParams.size:3}
	cls.params[LedParams.colors] = ["green", "red", "blue", "yellow", "magenta", "cyan", "white"]
        cls.params[LedParams.hs_values] = {"green": ((0.33, 0.8), (0.15, 0.2)),
                                           "red": ((0.0, 0.8), (0.15, 0.2)),
                                           "blue": ((0.66, 0.8), (0.15, 0.2)),
                                           "yellow": ((0.16, 0.8), (0.15, 0.2)),
                                           "magenta": ((0.83, 0.8), (0.15, 0.2)),
                                           "cyan": ((0.5, 0.8), (0.15, 0.2)),
                                           "white": ((0.5, 0.25), (0.5, 0.25))}
    @classmethod
    def tearDownClass(cls):
        pass

    def generate_image(self, color, value=0.5):
        hs = self.params[LedParams.hs_values][color]
        rgb = colorsys.hsv_to_rgb(hs[0][0], hs[0][1], value)
        img_size = np.concatenate([self.params[LedParams.size]*np.ones(2, dtype='uint8'), np.array([1])])
        base_img = np.ones(img_size)*255
        img = np.concatenate([rgb[2]*base_img, rgb[1]*base_img, rgb[0]*base_img], axis=2)
        return  img

    def test_exact_color_detection(self):
        for color in self.params[LedParams.colors]:
            img = self.generate_image(color)
            center = np.array([self.params[LedParams.size]/2, self.params[LedParams.size]/2])
            led = Led(center, img, self.params)
            self.assertEqual(color, led.color, "Detected color is wrong:  %s <-> %s(%s)"%(color, led.color, led.raw_color))

    def test_offset_color_detection(self):
        offset = np.mgrid[0:0.1:5j, 0:0.1:5j, 0:0.1:5j]
        for color in self.params[LedParams.colors]:
            for i in range(0, offset.shape[1]):
                for j in range(0, offset.shape[2]):
                    for k in range(0, offset.shape[3]):
                        img = self.generate_image(color)
                        img += offset[:,i,j,k]
                        center = np.array([self.params[LedParams.size]/2, self.params[LedParams.size]/2])
                        led = Led(center, img, self.params)
                        self.assertEqual(color, led.color, "Detected color is wrong:  %s <-> %s(%s)"%(color, led.color, led.raw_color))

    def test_different_illumination_color_detection(self):
        for v in np.linspace(0.1, 1, 10):
            for color in self.params[LedParams.colors]:
                img = self.generate_image(color, v)
                center = np.array([self.params[LedParams.size]/2, self.params[LedParams.size]/2])
                led = Led(center, img, self.params)
                self.assertEqual(color, led.color, "Detected color is wrong:  %s <-> %s(%s)"%(color, led.color, led.raw_color))
