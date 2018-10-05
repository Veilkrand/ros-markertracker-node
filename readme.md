# Marker Tracker ROS node

Node to detect and estimate pose of ArUCO markers from a subscribed raw image topic from the camera.

## TODO

-- Define message format with several geometry. Not include euler??
-- Check latency of messages. Fix buffer size??


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
 
## Visualization

rosrun image_view image_view image:=markertracker_node/image_result

## Output marker object
```
    {
        'marker_id': 10,
        'tvec': array([-17.79854065,  -4.45784617,  59.2286418 ]),
        'rvec': array([ 2.9659655 ,  0.19115458, -0.55335781]), 
        'euler': array([ 3.04171163,  0.3740164 ,  0.10980613])
    }
```
