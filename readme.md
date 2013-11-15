This utility allows you to read skeleton data from a number of Kinect sensors connected to your computer, and synthesize and output skeleton data from all sensors in a global coordinate frame. This means you can read skeleton data over a larger area and get relatively accurate positioning data through the whole space.

It wraps the Kinect Sensor class into a LocatedSensor class, which applies position and rotation transformations to the skeleton data, which is further wrapped up in the VisualKinectUnit class, which allows for easy visualization when paired with the KinectViewport custom control, which provides a RGB stream and skeleton visualization overlay, as well as controls to adjust the offset parameters of the Kinect units you have set up.


A simple utility that allows viewing Kinect video stream and skeletons. It also sends skeleton data as OSC.
There are several options:
1. You can send data as raw Kinect data or as percent of the width and height of viewing area 
(z remains raw data in meters)
2. You can send data as a single string in this format:
"Joint name|Joint number:.56|.45|1.4, ... more joints"
3. Or you can send the data as bundles of x, y, z data for each joint.
4. You can choose the port number
5. You can choose to view the video or skeletons (or not)
6. You can choose to track only the nearest skeleton
7. You can enable seated mode which sends a skeleton with no legs

The data comes in this order:
    1    hipCenter
    2    spine
	3    shoulderCenter
	4    head
	5    shoulderLeft
	6    elbowLeft
	7    wristLeft
	8    handLeft
	9    shoulderRight
	10   elbowRight
	11	 wristRight
	12   handRight
	13   hipLeft
	14   kneeLeft
	15   ankleLeft
	16   footLeft
	17   hipRight
	18   kneeRight
	19   ankleRight
	20   footRight