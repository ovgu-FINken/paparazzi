"""
    Region-of-Interest (RoI) test suite
"""
import unittest
from numpy import array, mgrid, linspace, repeat, concatenate, zeros, ones, minimum, maximum, append

PKG = 'tracking'

import rospkg
import sys
sys.path = [rospkg.RosPack().get_path(PKG)+"/src"]+sys.path


from ROI import RoIParams, RoI

class TestRoi(unittest.TestCase):
    """
        Region-of-Interest (RoI) test cases

        Initialization test: checks for default values
        Update test: checks updating RoI with new position
        Single step test: checks RoI prediction for a single step
        Crop test: test image extraction based on RoI prediction
    """

    @classmethod
    def setUpClass(cls):
        cls.params = {RoIParams.initial_size: 10,
                      RoIParams.size_increment: 10}

    @classmethod
    def tearDownClass(cls):
        pass

    def test_roi_init(self):
        """
            Tests default value initialization
        """
        roi = RoI(self.params)

        self.assertTrue(roi.position == None,\
        "RoI wrongly initialized: position not None")

        self.assertTrue(roi.speed == None,\
        "RoI wrongly initialized: speed not None")
        
	self.assertTrue(roi.estimation == None,\
        "RoI wrongly initialized: estimation not None")


    def test_roi_update(self):
        """
            Tests updating RoI with new position

            Checks position actualization, speed and direction estimation
        """
        speed = mgrid[0:1:5j, 0:1:5j]
        for i in range(0, speed.shape[1]):
            for j in range(0, speed.shape[2]):
                roi = RoI(self.params)
                value = speed[:, i, j]
                roi.update(value)

                self.assertTrue((roi.position == value).all(),\
                "RoI wrongly updated: position wrong %s"%roi.position)

                self.assertTrue(roi.speed == None,\
                "RoI wrongly updated: speed not None")
                
		roi.update(value)

                self.assertTrue((roi.position == value).all(),\
                "RoI wrongly updated: position wrong %s"%roi.position)

                self.assertTrue((roi.speed == array([0, 0])).all(),\
                "RoI wrongly updated: speed not (0, 0)")

    def test_roi_crop(self):
        """
            Tests croping of image based on current prediction of RoI
            :todo not implemented yet
        """
        size = self.params[RoIParams.initial_size]
        x_size = 100
        y_size = 50
        x = linspace(0, x_size, x_size, dtype="uint8").reshape(x_size, 1, 1)
        y = linspace(0, y_size, y_size, dtype="uint8").reshape(1, y_size, 1)
        x = repeat(x, y_size, axis=1)
        y = repeat(y, x_size, axis=0)
        z = zeros([x_size, y_size, 1], dtype="uint8")
        img = concatenate([x, y, z], axis=2)
        speed = mgrid[0:1:5j, 0:1:5j]
        for i in range(0, speed.shape[1]):
            for j in range(0, speed.shape[2]):
                roi = RoI(self.params)
                roi.update(array([0, 0]))
                roi.update(speed[:, i, j])
                cropped_img = roi.crop(img)
                start = (roi.position + roi.speed).astype("int") - array([int(size/2), int(size/2)])
                end = start + size*ones(2)
                start = maximum(start, zeros(2))
                end = minimum(end, array([x_size, y_size]))
                shape = append(end-start, 3)
                self.assertTrue((array(cropped_img.shape) == shape).all(), \
                    "cropped image size is wrong: %s <-> %s"%(cropped_img.shape, shape))
                ref_img = img[start[0]:end[0], start[1]:end[1], :]
                self.assertTrue((array(cropped_img) == ref_img).all(),\
                    "cropped image contents is wrong:\n %s\n%s"%(cropped_img, ref_img))
