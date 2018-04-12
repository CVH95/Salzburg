# ROVI 2 Final Project: OBJECT AVOIDANCE

## Download repo

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/CVEarp/ROVI2_Object_Avoidance/settings/collaboration
```

To build ROS packages (in $ROVI2\_Object\_Avoidance/ROS\_pkgs/)

```
$ catkin_make
```

RWStudio & Non-ROS programs have to b build individualy.

## Robot connection setup:

```
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

```
$ hostname -I
$ ping 192.168.100.52
$ ping 192.168.100.2
```

4.  Edit .xml parameter file with the IPs of the robot and the laptop 

```
$ source /opt/ros/kinetic/setup.bash
$ source catkin_ws/devel/setup.bash
$ vim catkin_ws/src/caros/hwcomponents/caros_universalrobot/launch/caros_universalrobot_param.xml

device_ip = 192.168.100.2
callbacl_ip = 192.168.100.25

$ roscore
```
5. Open new terminal and launch test example:

```
$ roslaunch caros_universalrobot simple_demo_using_move_ptp.test
```

## Network connection setup

#### On the Robot's computer:

Check ROS MASTER URI:

```
$ roscore
ROS_MASTER_URI=http://<IP>:11311/
```

IP should be 192.168.100.52 (that of the workcell)

Then, export URI and IP:

```
$ export ROS_MASTER_URI=http://192.168.100.52:11311/
$ export ROS_IP=192.168.100.52
```

#### On the laptop:

Export URI and IP (THIS HAS TO BE DONE FOR ANY NEW TERMINAL USED TO RUN ROS COMMANDS) 

```
$ export ROS_MASTER_URI=http://192.168.100.52:11311/
$ export ROS_IP=192.168.100.25
```

## Stereo Camera Connection

Installing the camera driver:

```
$ sudo apt install ros-kinetic-pointgrey-camera-driver
```

#### Launching the driver (in the Robot computer):

```
$ source /opt/ros/kinetic/setup.bash
$ roscore
```

New terminal

```
$ roslaunch pointgrey_camera_driver bumblebee.launch
```

#### Visualizing (Robot's computer or any other one in the network):

Run first /rostopic list/ to check if topics launched on Lab computer can be seen in the laptop (URI configuration). 

```
$ rostopic list
$ rosrun image_view image_view image:=/camera/right/image_raw
$ rosrun image_view image_view image:=/camera/left/image_raw
```

