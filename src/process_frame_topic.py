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
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseArray, Pose  #, Point32
from sensor_msgs.msg import Image #, CameraInfo
from tf.transformations import quaternion_from_euler


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
        # TODO: Setup all remain params for Aruco

        self._camera_frame_id = rospy.get_param("~camera_frame_id", default="base_link")

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
        self.marker_viz_pub = rospy.Publisher(_result_markers_viz_topic, PoseArray, queue_size=1)
        # Marker results publishers
        self.marker_pub = rospy.Publisher(_result_poses_topic, MarkersArray, queue_size=100)
        # Result publisher
        self.result_pub = rospy.Publisher(_result_poses_topic, Image, queue_size=1)


        ## Rospy loop
        r = rospy.Rate(30) # 10 Hz
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
            self._create_and_publish_markers_msg_from_results(poses, corners, image.header.stamp, self._camera_frame_id)
            # self._create_viz_markers_from_results(poses)

        if cv_image_result is not None: image = cv_image_result

        ## Publish Image
        if self.publish_topic_image_result:
            self._publish_cv_image(image)

    def _create_and_publish_markers_msg_from_results(self, poses, corners, image_timestamp, camera_frame_id):
        #print(poses)

        markers_array = MarkersArray()
        markers_array.header.stamp = rospy.Time.now()
        markers_array.camera_frame_stamp = image_timestamp

        # Only if Viz is enable
        viz_pose_array = PoseArray()
        #viz_pose_array.header.stamp = rospy.Time.now()
        viz_pose_array.header.stamp = image_timestamp
        viz_pose_array.header.frame_id = camera_frame_id


        #rospy.loginfo(poses[0]['euler'])

        # TODO: This iteration is extremely slow. Slower than the image generation. One marker Hz=20, list iteration Hz=8
        for e in poses:

            marker = Marker()
            marker.id = int(e['marker_id'])
            marker.camera_frame_stamp = image_timestamp # Is it necessary to include this?
            marker.translation_vector = Point(e['tvec'][0], e['tvec'][1], e['tvec'][2])
            marker.rotation_vector = Point(e['rvec'][0],e['rvec'][1],e['rvec'][2])
            marker.rotation_euler = Point(e['euler'][0],e['euler'][1],e['euler'][2])
            # marker.corners = corners[0] # TODO: needs implementation

            # PoseWithCovarianceStamped
            marker.pose_cov_stamped = self._create_pose_cov_stamped(marker, camera_frame_id, image_timestamp)

            markers_array.marker.append(marker)
            # Viz
            viz_pose_array.poses.append(marker.pose_cov_stamped.pose.pose)



        # Publish marker array
        self.marker_pub.publish(markers_array)

        # Only if viz is enables
        self.marker_viz_pub.publish(viz_pose_array)


    def _create_pose_cov_stamped(self, marker, frame_id, camera_frame_stamp):
        """
        From OpenCV vectors to ROS Pose object
        :param marker:
        :param frame_id:
        :return:
        """

        pose_stamped = PoseWithCovarianceStamped()

        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = camera_frame_stamp

        # pose_stamped.pose.pose.position = marker.translation_vector
        # Change units from cm to m. Change axis representation
        pose_stamped.pose.pose.position.x = marker.translation_vector.z / 100.0 #z
        pose_stamped.pose.pose.position.y = - marker.translation_vector.x / 100.0 #x
        pose_stamped.pose.pose.position.z = - marker.translation_vector.y / 100.0 #y


        """
        marker.rotation_euler.x = 0.0
        marker.rotation_euler.y = 0.0
        marker.rotation_euler.z = 0.0

        """
        _quaternion = quaternion_from_euler(marker.rotation_euler.x, #z
                                            marker.rotation_euler.y, #x
                                            marker.rotation_euler.z, #y
                                            axes='ryzx' ) # ryzx rxzy rzxy ryxz

        pose_stamped.pose.pose.orientation.x = _quaternion[0]
        pose_stamped.pose.pose.orientation.y = _quaternion[1]
        pose_stamped.pose.pose.orientation.z = _quaternion[2]
        pose_stamped.pose.pose.orientation.w = _quaternion[3]


        """
        pose_stamped.pose.pose.orientation.x = 0
        pose_stamped.pose.pose.orientation.y = 0
        pose_stamped.pose.pose.orientation.z = 0
        pose_stamped.pose.pose.orientation.w = 1
        """

        # TODO: simple covariances... Assign better for aruco and/or camera??
        _covariance = [ 1e-6, 0, 0, 0, 0, 0,
                        0, 1e-6, 0, 0, 0, 0,
                        0, 0, 1e-6, 0, 0, 0,
                        0, 0, 0, 1e-3, 0, 0,
                        0, 0, 0, 0, 1e-3, 0,
                        0, 0, 0, 0, 0, 1e-3 ]

        pose_stamped.pose.covariance = _covariance

        return pose_stamped

if __name__ == '__main__':

    _node_name = 'markertracker_node'

    print('* {} starting... '.format(_node_name), end="")

    rospy.init_node(_node_name, anonymous=True)

    PublisherSubscriberProcessFrame(_node_name)

    print('Ready.')

    rospy.spin()

