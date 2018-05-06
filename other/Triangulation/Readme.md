# Stereopsis

This folder is about performing the triangulation based on the intrinsic calibration data obtained with ROS and the extrinsic calibration performed with the help of a ruler and tweaking a simulated camera in RobWorkStudio.

The ROS calibration file containing the camera intrinsics and distortion parameters is in calibration.tar.gz
The projection matrix is not used since we need to construct our own projection matrix based on the scene.

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

