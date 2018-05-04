# ROVI 2 Final Project: OBJECT AVOIDANCE

This project was developed during the course of Robotics and Computer Vision 2 (spring 2018), at University of Southern Denmark, by Carlos, Richárd, Sergi and Mathesh. The setup is a WorkCell composed by a UR5 robot arm mounted on a table with a BumbleBee 2 stereo camera mounted on a corner. The project consisted on an application for dynamic path planning capable of real time replanning when an obstacle is placed interrupting its route, from an initial configuration q\_start to a final one q\_end (like in a pick & place). As obstacle object we used an small red ball. 

## A. Download and build repository

```sh
$ cd ~/catkin_ws/src/
$ git clone https://github.com/CVEarp/ROVI2_Object_Avoidance.git
```

To build ROS packages (in ROVI2\_Object\_Avoidance/ROS\_pkgs/)

```sh
$ catkin_make
```

RWStudio & Non-ROS programs have to b build individualy.

## B. Robot connection setup.

```sh
$ sudo vim /etc/hosts
```

1. Add workcell and robot IP:

```
192.168.100.52    WC2
192.168.100.2     UR2
```

2. Disable WiFi and creare ethernet connection with:

```
Name: roviWorkcell
IP: 192.168.100.25
Netmask: 255.255.255.0
Gateway: -
```

3. Check IP and ping both the robot and the workcell

```sh
$ hostname -I
$ ping 192.168.100.52
$ ping 192.168.100.2
```

4.  Edit .xml parameter file with the IPs of the robot and the laptop 

```sh
$ source /opt/ros/kinetic/setup.bash
$ source catkin_ws/devel/setup.bash
$ vim catkin_ws/src/caros/hwcomponents/caros_universalrobot/launch/caros_universalrobot_param.xml

device_ip = 192.168.100.2
callbacl_ip = 192.168.100.25

$ roscore
```
5. Open new terminal and launch test example:

```sh
$ roslaunch caros_universalrobot simple_demo_using_move_ptp.test
```

## C. Network connection setup.

#### On the Robot's computer:

Check ROS MASTER URI:

```sh
$ roscore
ROS_MASTER_URI=http://<IP>:11311/
```

IP should be 192.168.100.52 (that of the workcell)

Then, export URI and IP:

```sh
$ export ROS_MASTER_URI=http://192.168.100.52:11311/
$ export ROS_IP=192.168.100.52
```

#### On the laptop:

Export URI and IP (THIS HAS TO BE DONE FOR ANY NEW TERMINAL USED TO RUN ROS COMMANDS) 

```sh
$ export ROS_MASTER_URI=http://192.168.100.52:11311/
$ export ROS_IP=192.168.100.25
```

## D. Stereo Camera Connection.

Installing the camera driver:

```sh
$ sudo apt install ros-kinetic-pointgrey-camera-driver
```

#### Launching the driver (in the Robot computer):

```sh
$ source /opt/ros/kinetic/setup.bash
$ roscore
```

New terminal

```sh
$ roslaunch pointgrey_camera_driver bumblebee.launch
```

#### Visualizing (Robot's computer or any other one in the network):

Run first /rostopic list/ to check if topics launched on Lab computer can be seen in the laptop (URI configuration). 

```
$ rostopic list
$ rosrun image_view image_view image:=/camera/right/image_raw
$ rosrun image_view image_view image:=/camera/left/image_raw
```

## E. USAGE.

The following sections describe the main architecture of the application, each node's functionality and how to use each of them.

### 1. CAROS UNIVERSALROBOT.

The main package. Launch the `caros_universalrobot.launch` to activate all the caros nodes, msgs and functionalities on which the Object Avoidance is based. Once all the robot and network connection setup is done, it is the first thing to launch:

```sh
$ roslaunch caros_universalrobot caros_universalrobot.launch
```

### 2. UR Caros Example.

Taken from the ROS Lecture exercise and slightly modified. Use it to set the robot in the initial position for the application. Remember to launch `caros_universalrobot.launch` first.

```sh
$ rosrun ur_caros_example ur_caros_example
```

### 3. Robot State Monitor.

This node simply subscribes to RobotSate's msgs to constantly monitor the state of the machine. Displays joint configuration, speed and moving status in the screen. To end this node's activity type ctrl+c in the terminal.

```sh
$ rosrun robot_state_monitoring robot_state_monitoring
```

### 4. Planner.

The planner node is the main one in charge of path planning and robot motion. Some features to mention about its implementation:

 - It is set up as a client of the `/move_servo_q` CAROS service. In order to move the robot, it requests the CAROS interface to do it by sending each configuraton vector Q in the trajectory calculated.
 - It is subscribed to `/robot_state` in order to be able to obtain instant information abour the current configuration state of the UR (needed for replanning). 

```sh
$ rosrun planner planner
```

### 5. Red Ball Detection.

Several nodes compose this package in which all the vision part is implemented. The sensor used is a BumbleBee 2 stereo camera.

 - Stereo Detection (for right and left images).
 - Stereo triangulation of the real world coordinates of the red ball.
 - Kalman Filter for ball's movement prediction.

#### 5.1. Right/Left Stereo Detection.

There are two nodes, one for each camera in the BumbleBee. As seen before, the sensor uses firewire connection, so it as to be plugged to the WorkStation computer. Therefore, before using it, it is required to export ROS MASTER URI and IP (`ROS_pkgs/export_master.sh`) to that of the WorkCell computer (sections C and D). These detectors are subscribed to both `image_raw` topics to get the input stream recorded by the camera. Then, after filtering the image and getting the coordinates of the ball, each of the nodes broadcast them into `red_ball_detection/right_image_coordinates` and `red_ball_detection/left_image_coordinates` topics.

```sh
$ rosrun stereo_detector_sinistro (left)
$ rosrun stereo_detector_destro (right)
```

#### 5.2. Stereo Triangulation

This node subscribes to the topics where detectors are publishing. Then, it calculates the 3D location by triangulating based on camera calibration parameters and pixel coordinates of the ball at both images. The 3D scene location of the ball (x, y, z real coordinates) are broadcasted then in the topic `/red_ball_detection/triangulated_ball_location`.

```sh
$ rosrun stereo_triangulation
```

#### 5.3. Kalman Filter


#### 5.4. Launch files

Two launch files are added to be able to run the detection nodes all at once and at the same time. One launches all the setup, while the other does not use the Kalman Filter.

```sh
$ roslaunch red_ball_detection static_detection.launch (no Kalman)
``` 
