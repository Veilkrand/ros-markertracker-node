#!/usr/bin/env python

from __future__ import print_function

import cv2
import cv2.aruco as aruco
import numpy as np
import math


class ArucoWrapper:

    def __init__(self, marker_length, camera_matrix, dist_coeffs, aruco_dictionary=0):

            # aruco_dict = cv2.aruco.getPredefinedDictionary( cv2.aruco.DICT_5X5_250 )
            # self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
            # self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) #0

            self.aruco_dict = aruco.getPredefinedDictionary(aruco_dictionary)
            self.parameters = aruco.DetectorParameters_create()

            self.marker_length = marker_length
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs


    def find_corners_from_image(self, image, draw_image=False):

        # TODO: implement param???
        image_is_gray = True
        if image_is_gray:
            image_gray = image
        else:
            image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        image_result = None

        aruco_corners, aruco_ids, aruco_rejected_points = aruco.detectMarkers(image_gray, self.aruco_dict, parameters=self.parameters)

        if draw_image is True and aruco_ids is not None and len(aruco_ids) > 0:
            image_result = aruco.drawDetectedMarkers(image, aruco_corners, aruco_ids)

        return image_result, aruco_corners, aruco_ids, aruco_rejected_points


    def find_poses_from_corners(self, aruco_ids, aruco_corners,image_color=None, draw_image=False):

        poses_result = None
        image_result = None

        if aruco_ids is not None and len(aruco_ids) > 0:

            poses_result = list()

            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(aruco_corners,
                                                                       self.marker_length,
                                                                       self.camera_matrix,
                                                                       self.dist_coeffs)

            for i in range(len(aruco_ids)):

                euler = self._get_euler_vector_from_rvec(rvecs[i])

                #x = tvecs[i][0][0]
                #y = tvecs[i][0][1]
                #z = tvecs[i][0][2]

                pose = { 'marker_id': aruco_ids[i][0],
                         #'tvec': (x, y, z),
                         'tvec': tvecs.flatten(),
                         'rvec': rvecs[i].flatten(),
                         'euler': euler }

                poses_result.append(pose)

                if draw_image is True and image_color is not None:
                    image_color = aruco.drawAxis(image_color,
                                                self.camera_matrix,
                                                self.dist_coeffs,
                                                rvecs[i], tvecs[i],
                                                self.marker_length / 2);


        return image_color, poses_result

    def get_poses_from_image(self,image, draw_image=False):
        _, aruco_corners, aruco_ids, _ = self.find_corners_from_image(image)
        return self.find_poses_from_corners(aruco_ids,aruco_corners,image,draw_image=draw_image)



    def _get_euler_vector_from_rvec(self, rvec):
        rmat, _ = cv2.Rodrigues(rvec)
        euler = rotationMatrixToEulerAngles(rmat)
        return euler

# TODO: move or refactor. Helper functions for calculating euler angles after a pose estimation
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])