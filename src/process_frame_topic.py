#!/usr/bin/env python
'''
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

Subscribe to a ROS image topic, transform to OpenCV image. And process the image.


D: [0.1915250639431566, -0.4623037094569031, 0.00130345932763652, -0.004691457734403636, 0.0]

K: [556.5250543909606, 0.0, 323.1341418711615,
    0.0, 516.0814264720717, 231.5051365870941,
    0.0, 0.0, 1.0]



'''

from __future__ import print_function

# import roslib
# roslib.load_manifest('my_package')

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from ArucoWrapper import ArucoWrapper



class ImageConverter:

    def __init__(self,input_image_topic, camera_info, result_image_topic=None, publish_topic_result=False):

        # Params
        # Topics
        self.input_image_topic = input_image_topic
        self.result_image_topic = result_image_topic
        self.publish_topic_result = publish_topic_result

        # Camera setup
        self.marker_lenght = 10

        '''
        TODO: implement
        fs = cv2.FileStorage("./calib_asus_chess/cam_calib_asus.yml", cv2.FILE_STORAGE_READ)
        fn = fs.getNode("camera_matrix")
        print(fn.mat())
        '''

        self.camera_matrix = camera_info.K
        self.dist_coeffs = camera_info.D

        print(self.camera_matrix)
        print(self.dist_coeffs)

        # Classes
        self.bridge = CvBridge()
        self.detector = ArucoWrapper(self.marker_lenght, self.camera_matrix, self.dist_coeffs)

        # Subscriber
        self.image_sub = rospy.Subscriber(input_image_topic, Image, self.process_frame, queue_size=1)

        # Publisher
        if publish_topic_result:
            self.image_pub = rospy.Publisher(result_image_topic, Image, queue_size=1)


    def process_frame(self, image):

        if image is None: return

        ## Preprocess
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        ## Process
        # Pose
        cv_image_result, poses = self.detector.get_poses_from_image(cv_image, draw_image=True)

        ## Publish
        if cv_image_result is not None and self.publish_topic_result:
            self.publish_cv_image(cv_image_result)

    def publish_cv_image(self, image):
        if image is None: return
        try:
            #pass
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):


    _node_name = 'markertracker_node'
    _result_image_topic = _node_name +'/image_result'

    print('* {} starting... '.format(_node_name), end="")

    rospy.init_node(_node_name, anonymous=True)

    _input_image_topic = rospy.get_param("~input_image_topic", None)
    _publish_topic_result = rospy.get_param("~publish_topic_result", False)
    _marker_ids = rospy.get_param("~marker_ids", None)
    _camera_info_topic = '/raspicam_node/camera_info'

    # Get camera info
    _camera_info = rospy.wait_for_message(_camera_info_topic, CameraInfo)
    # print(camera_info.height)

    # Call image converter
    ic = ImageConverter(_input_image_topic, _camera_info, _result_image_topic, publish_topic_result=_publish_topic_result)

    print('Ready.')

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
