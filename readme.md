# Marker Tracker ROS node

Node to detect and estimate pose of ArUCO markers from a subscribed raw image topic from the camera.
In v2 a ROS Pose with covariance stamped message is also provided. Local pose is stamped with the original camera frame stamp.


## TODO

-- Implement blacklist and whitelist for aruco Ids. 17 is easily confused for 4x4 and background noise
-- Finish message implementation
-- Check real latency and delay. For result packages and initial image frames: how long since the capture of the raw image from camera.
-- Publish stamped pose with covariances with original camera image stamp.
-- Add original covariances from local ego pose. 
-- Implement full visualization for rviz? Not here.
-- Optimize Aruco classes. Include all params for aruco.


## Inputs Parameters. TODO: Update to latest inputs

* image_topic  [Image topic]: (Empty) Input image topic to process from camera frames.
* max_n_markers [Integer]: (10) Maximum number of markers to detect. Optimization param.
* marker_ids [Array:Integer]: (Empty) Array for ids for pose estimation, only included ids will be estimated. Empty value will estimate all ids.
* show_image_topic [Boolean]: (false) Image topic with marker detection for debug and visualization purposes.
* image_is_gray [Boolean]: (false) Is the image already converted to grayscale? Optimization param.
* image_threshold [Integer]: (0) Value from 0-255 to applied a processing threshold to improve marker detection. Default 0 value is deactivated. 

## Outputs Topics

 * /markertracker_node/detected_markers
 * /markertracker_node/image_result (only if `show_image_topic==True`)
 
## Result Visualization

`rqt_image_view`
`rosrun image_view image_view image:=markertracker_node/image_result`


## Output marker object. TODO: This is Outdated
```
    {
        'marker_id': 10,
        'tvec': array([-17.79854065,  -4.45784617,  59.2286418 ]),
        'rvec': array([ 2.9659655 ,  0.19115458, -0.55335781]), 
        'euler': array([ 3.04171163,  0.3740164 ,  0.10980613])
    }
```

## Rosbag
Save only camera images topics:
`rosbag record -O camera_topic /iris/usb_cam/image_raw`
Play: 
`rosbag play camertopic_10s.bag`
