# Marker Tracker ROS node
> Alberto S. Naranjo Galet. 2018

Node to detect and estimate pose of ArUCO markers from a subscribed raw image topic from the camera.
In v2 a ROS Pose with covariance stamped message is also provided. Local pose is stamped with the original camera frame stamp.


## TODO

-- Implement blacklist and whitelist for aruco Ids. 17 is easily confused for 4x4 and background noise
-- Add original covariances from local ego pose. 
-- Optimize Aruco classes. Include all params for aruco.
-- Optimize OpenCV euler to ros quaternion


## Inputs Parameters. TODO: Update to latest inputs

* image_topic  [Image topic]: (Empty) Input image topic to process from camera frames.
* max_n_markers [Integer]: (10) Maximum number of markers to detect. Optimization param.
* marker_ids [Array:Integer]: (Empty) Array for ids for pose estimation, only included ids will be estimated. Empty value will estimate all ids.
* show_image_topic [Boolean]: (false) Image topic with marker detection for debug and visualization purposes.
* image_is_gray [Boolean]: (false) Is the image already converted to grayscale? Optimization param.
* image_threshold [Integer]: (0) Value from 0-255 to applied a processing threshold to improve marker detection. Default 0 value is deactivated. 

## Outputs Topics

 * */markertracker_node/image_result* (only if `show_image_topic==True`)
 * */markertracker_node/poses* With correct frame id and timestamp 
 * */markertracker_node/visualization_markers*
 * */markertracker_node/gate_markers*

 
## Result Visualization

`rqt_image_view`
`rosrun image_view image_view image:=markertracker_node/image_result`


## Rosbag
Save only camera images topics:
`rosbag record -O camera_topic /iris/usb_cam/image_raw`
Play: 
`rosbag play camertopic_10s.bag`
