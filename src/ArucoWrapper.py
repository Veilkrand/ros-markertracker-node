#!/usr/bin/env python

from __future__ import print_function

import cv2
import cv2.aruco as aruco

class ArucoWrapper:

    def __init__(self, marker_lenght, camera_matrix, dist_coeffs):
            # aruco_dict = cv2.aruco.getPredefinedDictionary( cv2.aruco.DICT_5X5_250 )
            # self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            self.parameters = aruco.DetectorParameters_create()

            self.marker_length = marker_lenght
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
        poses_result = list()

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

                x = tvecs[i][0][0]
                y = tvecs[i][0][1]
                z = tvecs[i][0][2]

                pose = { 'marker_id': aruco_ids[i],
                         'tvec' : (x,y,z),
                         'rvec' : rvecs[i],
                         'euler' : euler }

                poses_result.append(pose)

                if draw_image is True and image_color is not None:
                    img_result = aruco.drawAxis(img_color,
                                                self.camera_matrix,
                                                self.dist_coeffs,
                                                rvecs[i], tvecs[i],
                                                self.marker_length / 2);


        return image_result, poses_result

    def get_poses_from_image(self,image, draw_image=False):
        _, aruco_corners, aruco_ids, _ = self.find_corners_from_image(image)
        return self.find_poses_from_corners(aruco_ids,aruco_corners,image,draw_image=draw_image)

    def _get_euler_vector_from_rvec(self, rvec):
        rmat, _ = cv2.Rodrigues(rvec)
        euler = rotationMatrixToEulerAngles(rmat)
        return euler