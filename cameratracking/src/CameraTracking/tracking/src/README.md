#### Color based copter tracking

This is an introduction to how the color based tracking software works. The file _pose.py_ contains all the
code for the ROS node that deals with tracking.

#### Overview

A birds eye view of the node code is presented below.

##### Initialization

- An opencv SimpleBlobDetector object is instantiated with the following parameters:
	- `filterByInertia = false`, to ignore inertia of shapes
	- `filterByConvexity = false`, to ignore convexity too
	- `filterByColor = false`, our objects have different colors so ignore this too
	- `filterByCircularity = false`, although our objects of interest are circular, they aren't always detected as such
	- `filterByArea = true`, this is the only parameter that seems to be constant in every case, so we'll filter by area
	- `minArea = 1`, an objects of as small as 1 pixel will be detected (after removing noise of course)
- The node is set to publish _/copter_ topic, which will include the pose of the copter.
- The node is set to publish _/copterimg_ topic, which will include the ROI of the image.

##### Definition of helper classes

The following classes are created as custom data types:

- _color_ : An enumerator defining colors blue, green, red and unknown.
- _LED_ : This class has simply x/y coordinates and a _color_ value.

##### Function definitions

Following is a list of functions and their purpose:

- _get\_keypoints_ : Takes raw RGB image, converts it to grayscale, applies threshold to remove noise and uses blob detector
  to get LEDs's keypoints.
- _get\_color_ : Takes raw image and a location (x/y) to detect color at that spot. This is done by averaging each channel
  (R, G and B) and then taking the max value channel as the color. Right now it only detects as red, green and blue but it will be
  to changed to detect more colors.
- _get\_theta_ : Takes a set of _LED_ objects and tries to guess the angle of the copter. It does so by drawing a line between two
  front LEDs, determinining the head and tail of this line by finding out on which side the back LEDs lie, and then returning its angle.
- _get\_center_ : Takes in a set of keypoints and returns their center point. It does so by connecting any two diagonal LEDs and returning
  the center of the line joining them. If the input are only two LEDs (diagonal or not), it will return the center of those two. That's why
  this value may jump around a bit.
- _search_ : This function creates a cluster of LEDs that are close together. It is not for direct use.
- _cluter_ : This function uses the _search_ function to get several clusters based on given keypoints. Can be used directly. It returns the
  result as array of arrays of indices of the given keypoints. I hope it makes sense!
- _callback_ : The callback function which is called everytime we get a new image. Its functionality is detailed later.

#### Steps

The steps taken to reach from start to end in the tracking process in the _callback_ are described below.

- Convert the input RAW image type from ROS message to a CV2 image.
- Get an ROI around the last found copter center. If its the first run, the whole image is the ROI.
- Call the _get\_keypoints_ function to get a list of all detected LEDs.
- Get the center point of these LEDs using the _get\_center_ function. Note that the current version requires that there is a SINGLE copter
  in the image. Support for multiple copter detection is in process.
- If the above step fails to get any center, the ROI is increased by a value of 100 pixels at each side, until it is successful.
- All the LEDs are then attached a color using _get\_color_ function.
- Copter's angle is got using _get\_theta_ function.
- The center and theta are packed into a _Pose2D_ object and published, alongwith the ROI image.
