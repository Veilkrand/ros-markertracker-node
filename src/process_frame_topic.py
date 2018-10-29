#!/usr/bin/env python
"""
http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

Subscribe to a ROS image topic, transform to OpenCV image. And process the image.


D: [0.1915250639431566, -0.4623037094569031, 0.00130345932763652, -0.004691457734403636, 0.0]

K: [556.5250543909606, 0.0, 323.1341418711615,
    0.0, 516.0814264720717, 231.5051365870941,
    0.0, 0.0, 1.0]
"""

from __future__ import print_function

# import roslib
# roslib.load_manifest('my_package')

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
#from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, Point32
from sensor_msgs.msg import Image, CameraInfo

from markertracker_node.msg import Marker, MarkersArray

import tf.transformations as tf # Needed?

from ArucoWrapper import ArucoWrapper


class PublisherSubscriberProcessFrame(object):
    """
     Subscribe to a image topic, call a process callback and publish results
    """

    def __init__(self, _node_name):

        self.node_name = _node_name
        # Publisher topics
        _result_image_topic = _node_name + '/image_result'
        _result_poses_topic = _node_name + '/markers'
        _result_markers_viz_topic = _node_name + '/visualization_marker'

        # Params
        _input_image_topic = rospy.get_param("~input_image_topic")
        _path_to_camera_file = rospy.get_param("~path_to_camera_file")
        self.marker_length = rospy.get_param("~marker_length")
        self.publish_topic_image_result = rospy.get_param("~publish_topic_image_result", False)
        _aruco_dictionary = rospy.get_param("~aruco_dictionary", 0)

        # Load camera file
        fs = cv2.FileStorage(_path_to_camera_file, cv2.FILE_STORAGE_READ)
        _camera_matrix = fs.getNode("camera_matrix").mat()
        _dist_coeffs = fs.getNode("distortion_coefficients").mat()
        fs.release()

        # Classes
        self.bridge = CvBridge()
        self.detector = ArucoWrapper(self.marker_length, _camera_matrix, _dist_coeffs, aruco_dictionary=_aruco_dictionary)

        # Subscriber
        self.image_sub = rospy.Subscriber(_input_image_topic, Image, self._callback, queue_size=1)
        self.latest_msg = None  # to keep latest received message
        self.new_msg_available = False

        ## Publishers
        # Image Publisher
        if self.publish_topic_image_result:
            self.image_pub = rospy.Publisher(_result_image_topic, Image, queue_size=1)
        # Marker viz publisher
        self.marker_viz_pub = rospy.Publisher(_result_markers_viz_topic, MarkerArray, queue_size=1)
        # Marker results publishers
        self.marker_pub = rospy.Publisher(_result_poses_topic, MarkersArray, queue_size=1)
        # Result publisher
        self.result_pub = rospy.Publisher(_result_poses_topic, Image, queue_size=1)


        ## Rospy loop
        r = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if self.new_msg_available:

                self.new_msg_available = False

                self.process_frame(self.latest_msg)

                #rospy.loginfo(msg)
                r.sleep()



    def _publish_cv_image(self, image):
        if image is None: return
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def _callback(self, data):
        self.latest_msg = data
        self.new_msg_available = True

    def process_frame(self, image):

        if image is None: return

        ## Preprocess
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        ## Pose and corners used
        cv_image_result, poses, corners = self.detector.get_poses_from_image(cv_image, draw_image=self.publish_topic_image_result)

        # process pose
        if poses is not None:
            self._create_and_publish_markers_msg_from_results(poses, corners)
            # self._create_viz_markers_from_results(poses)

        if cv_image_result is not None: image = cv_image_result

        ## Publish Image
        if self.publish_topic_image_result:
            self._publish_cv_image(image)

    def _create_and_publish_markers_msg_from_results(self, poses, corners):
        #print(poses)

        markers_array = MarkersArray()
        markers_array.header.stamp = rospy.Time.now()

        for e in poses:

            marker = Marker()
            marker.id = int(e['marker_id'])
            marker.translation_vector = Point(e['tvec'][0], e['tvec'][1], e['tvec'][2])
            marker.rotation_vector = Point(e['rvec'][0],e['rvec'][1],e['rvec'][2])
            marker.rotation_euler = Point(e['euler'][0],e['euler'][1],e['euler'][2])
            # marker.corners = corners[0] implement
            markers_array.marker.append(marker)

        self.marker_pub.publish(markers_array)




    # Marker visualization test. It didn't work very well
    def _create_viz_markers_from_results(self, results):

        if results is None:
            return

        marker_array = MarkerArray()

        for m in results:

            marker = Marker()

            marker.header.frame_id = self.node_name
            marker.header.stamp = rospy.Time.now()
            q = tf.quaternion_from_euler(m['euler'][0], m['euler'][1], m['euler'][2], 'ryxz')
            marker.id = m['marker_id']
            marker.ns = self.node_name
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = q[0] #1
            marker.pose.orientation.x = q[1]
            marker.pose.orientation.y = q[2]
            marker.pose.orientation.z = q[3]
            marker.pose.position.x = m['tvec'][0]/100
            marker.pose.position.y = m['tvec'][1]/100
            marker.pose.position.z = m['tvec'][2]/100

            marker_array.markers.append(marker)

        # Publish
        self.marker_viz_pub.publish(marker_array)


if __name__ == '__main__':

    _node_name = 'markertracker_node'

    print('* {} starting... '.format(_node_name), end="")

    rospy.init_node(_node_name, anonymous=True)

    PublisherSubscriberProcessFrame(_node_name)

    print('Ready.')

    rospy.spin()

