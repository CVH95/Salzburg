# ROVI 2 Final Project: OBJECT AVOIDANCE

By Carlos, Richárd, Sergi and Mathesh.

## Download and build repository

```sh
$ cd ~/catkin_ws/src/
$ git clone https://github.com/CVEarp/ROVI2_Object_Avoidance.git
```

To build ROS packages (in ROVI2\_Object\_Avoidance/ROS\_pkgs/)

```sh
$ catkin_make
```

RWStudio & Non-ROS programs have to b build individualy.

## Robot connection setup.

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

## Network connection setup.

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

## Stereo Camera Connection.

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

## USAGE.

The following sections describe the main architecture of the application, each node's functionality and how to use each of them.

#### 1. CAROS UNIVERSALROBOT.

The main package. Launch the `caros_universalrobot.launch` to activate all the caros nodes, msgs and functionalities on which the Object Avoidance is based. Once all the robot and network connection setup is done, it is the first thing to launch:

```sh
$ roslaunch caros_universalrobot caros_universalrobot.launch
```

#### 2. UR Caros Example.

Taken from the ROS Lecture exercise and slightly modified. Use it to set the robot in the initial position for the application. Remember to launch `caros_universalrobot.launch` first.

```sh
$ rosrun ur_caros_example ur_caros_example
```

#### 3. Robot State Monitor.

This node simply subscribes to RobotSate's msgs to constantly monitor the state of the machine. Displays joint configuration, speed and moving status in the screen. To end this node's activity type ctrl+c in the terminal.

```sh
$ rosrun robot_state_monitoring robot_state_monitoring
```

#### 4. Planner.

The planner node is the main one in charge of path planning and robot motion. Some features to mention about its implementation:

 - It is set up as a client of the `/move_servo_q` CAROS service. In order to move the robot, it requests the CAROS interface to do it by sending each configuraton vector Q in the trajectory calculated.
 - It is subscribed to `/robot_state` in order to be able to obtain instant information abour the current configuration state of the UR (needed for replanning). 

```sh
$ rosrun planner planner
```
