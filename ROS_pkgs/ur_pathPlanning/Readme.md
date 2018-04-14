# HOW TO EXECUTE

### Network connection

Just requires connecting to the robot, not the workcell.
(enough with pluggin the ethernet, no need to export ROS MASTER URI and IPs)

```
$ ping 192.168.100.2
```

## Running the programm

It uses caros UR msgs:

```
$ source /opt/ros/kinetic/setup.bash
$ source catkin_ws/devel/setup.bash
$ roslaunch caros_universalrobot caros_universalrobot.launch
$ rosrun ur_pathPlanning ur_pathPlanning
```
