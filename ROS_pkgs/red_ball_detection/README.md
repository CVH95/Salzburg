# DETECTOR PACKAGE

Uses a Bumblebee stereo camera to track the obstacle (red ball) and triangulate its 3D coordinates with respect to the UR5 base frame. Three main nodes:

 - Detector 
 - Stereo triangulation
 - Kalman tracking 

Comunnication bewteen nodes done through publisher/subscriber protocol.

#### List of ROS topics launched by /pointgrey\_camera\_driver/bumblebee.launch

```sh
$ rostopic list

/camera/bumblebee_nodelet/parameter_descriptions
/camera/bumblebee_nodelet/parameter_updates
/camera/bumblebee_nodelet_manager/bond
/camera/image_exposure_sequence
/camera/image_proc_debayer_left/parameter_descriptions
/camera/image_proc_debayer_left/parameter_updates
/camera/image_proc_debayer_right/parameter_descriptions
/camera/image_proc_debayer_right/parameter_updates
/camera/left/camera_info
/camera/left/image_color
/camera/left/image_color/compressed
/camera/left/image_color/compressed/parameter_descriptions
/camera/left/image_color/compressed/parameter_updates
/camera/left/image_color/compressedDepth
/camera/left/image_color/compressedDepth/parameter_descriptions
/camera/left/image_color/compressedDepth/parameter_updates
/camera/left/image_color/theora
/camera/left/image_color/theora/parameter_descriptions
/camera/left/image_color/theora/parameter_updates
/camera/left/image_mono
/camera/left/image_mono/compressed
/camera/left/image_mono/compressed/parameter_descriptions
/camera/left/image_mono/compressed/parameter_updates
/camera/left/image_mono/compressedDepth
/camera/left/image_mono/compressedDepth/parameter_descriptions
/camera/left/image_mono/compressedDepth/parameter_updates
/camera/left/image_mono/theora
/camera/left/image_mono/theora/parameter_descriptions
/camera/left/image_mono/theora/parameter_updates
/camera/left/image_raw
/camera/left/image_raw/compressed
/camera/left/image_raw/compressed/parameter_descriptions
/camera/left/image_raw/compressed/parameter_updates
/camera/left/image_raw/compressedDepth
/camera/left/image_raw/compressedDepth/parameter_descriptions
/camera/left/image_raw/compressedDepth/parameter_updates
/camera/left/image_raw/theora
/camera/left/image_raw/theora/parameter_descriptions
/camera/left/image_raw/theora/parameter_updates
/camera/right/camera_info
/camera/right/image_color
/camera/right/image_color/compressed
/camera/right/image_color/compressed/parameter_descriptions
/camera/right/image_color/compressed/parameter_updates
/camera/right/image_color/compressedDepth
/camera/right/image_color/compressedDepth/parameter_descriptions
/camera/right/image_color/compressedDepth/parameter_updates
/camera/right/image_color/theora
/camera/right/image_color/theora/parameter_descriptions
/camera/right/image_color/theora/parameter_updates
/camera/right/image_mono
/camera/right/image_mono/compressed
/camera/right/image_mono/compressed/parameter_descriptions
/camera/right/image_mono/compressed/parameter_updates
/camera/right/image_mono/compressedDepth
/camera/right/image_mono/compressedDepth/parameter_descriptions
/camera/right/image_mono/compressedDepth/parameter_updates
/camera/right/image_mono/theora
/camera/right/image_mono/theora/parameter_descriptions
/camera/right/image_mono/theora/parameter_updates
/camera/right/image_raw
/camera/right/image_raw/compressed
/camera/right/image_raw/compressed/parameter_descriptions
/camera/right/image_raw/compressed/parameter_updates
/camera/right/image_raw/compressedDepth
/camera/right/image_raw/compressedDepth/parameter_descriptions
/camera/right/image_raw/compressedDepth/parameter_updates
/camera/right/image_raw/theora
/camera/right/image_raw/theora/parameter_descriptions
/camera/right/image_raw/theora/parameter_updates
/camera/temp
/diagnostics
/rosout
/rosout_agg
```

Initially, the detector program should be subscribed to `/image_raw` of both cameras (left and right) and convert it into `cv::Mat`. From there, perform detection algorithm and publish 2D pixel coordinates of the center of the ball in both images into another topic (via red_ball_detection/ballCentrum.msg), created for communication with the triangulation node. 
The triangulation node calculates the 3D position of the ball and publishes the coordinates into a third topic (via red_ball_detection/ballToRobotBase.msg) that the planner reads.
