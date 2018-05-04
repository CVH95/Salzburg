# DETECTOR PACKAGE

Uses a Bumblebee stereo camera to track the obstacle (red ball) and triangulate its 3D coordinates with respect to the UR5 base frame. Four main nodes:

 - Detector (right image): `stereo_detector_destro`
 - Detector (left image): `stereo_detector_sinistro`
 - Stereo triangulation: `stereo_triangulation`
 - Kalman tracking: `(Not implemented yet)` 

Communication bewteen nodes done through publisher/subscriber protocol.

#### List of ROS topics launched by /pointgrey\_camera\_driver/bumblebee.launch

In order to work with the camera, it is necessary to export ROS\_MASTER\_URI and ROS\_IP to that of the WorkCell's comuter (and launch the bumblebee driver in that computer aswell).
Remember to repeat this for all terminals in which vision nodes will be launch.

```sh
$ export ROS_MASTER_URI=http://192.168.100.52:11311/
$ export ROS_IP=192.168.100.25
```

ROS topics launched by bumblebee driver: 

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

Initially, the detector program should be subscribed to `/image_raw` of both cameras (left and right) and convert it into `cv::Mat`. From there, perform detection algorithm and publish 2D pixel coordinates of the center of the ball in both images into another topic (via red\_ball\_detection/ballCentrum.msg), created for communication with the triangulation node. 
The triangulation node calculates the 3D position of the ball and publishes the coordinates into a third topic (via red\_ball\_detection/ballToRobotBase.msg) that the planner reads.

#### Image Feature Detectors

The same algorithm is applied to both images perceived by the Bumblebee 2 camera. The main feature that we are trying to detect is the center of the Red Ball we use as obstacle. To do that, separately we subscribe the right image detector to the topic `/camera/right/image_raw` , while the left detector is subscribed to `/camera/left/image_raw` topic to receive images. This gives the raw input (uncorrupted) to the program. To recognize the ball, we implemented a detector based on color segmentation plus morphological manipulation of the image to produce a binary video stream with all background in black, and (hopefully) a shape similar to a circle where the ball is. Then, we approximate the ball's shape by a minimum enclosing circle acting as bounding box, and use the center of this bounding circle as the ball center. 

The pixel coordinates of the ball are published in the topics: 

 - Left image: `/red_ball_detection/left_image_location`
 - Right image: `/red_ball_detection/right_image_location`


#### Stereopsis

This node is about performing the triangulation based on the intrinsic calibration data obtained with ROS and the extrinsic calibration performed with the help of a ruler and tweaking a simulated camera in RobWorkStudio.

The ROS calibration file containing the camera intrinsics and distortion parameters is in calibration.tar.gz The projection matrix is not used since we need to construct our own projection matrix based on the scene.

The triangulation program is the solution from lecture 2 of vision. The input at this moment is a txt file containing calibration data (I stitched this file from my ROS and my own measurement), the left and the right image and the two coordinates to triangulate.

The calibration file contains for each camera:

  - number of cameras
  - resolution of pictures
  - camera intrinsic matrix (3x3)
  - distortion parameters (4x1)
  - rotation matrix from the base frame of the robot and the camera frame (3x3)
  - translation vector from the base frame of the robot and the camera frame (3x1)

The output of the program is the triangulated 3D position measured in the base frame.

In addition to modifying the program a bit, the right parameters are needed to be found. It's not ready yet.

 
