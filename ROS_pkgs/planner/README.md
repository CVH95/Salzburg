# PLANNER NODE

Implementation of Anytime Dynamic RRT path planning algorithm for Object Avoidance with a UR5 arm.

### ROS nodes:

```sh
charlie@Asgard:~$ rostopic list
/caros_universalrobot/caros_node/caros_node_state
/caros_universalrobot/caros_serial_device_service_interface/robot_state
/rosout
/rosout_agg

charlie@Asgard:~$ rosservice list
/caros_universalrobot/caros_node/recover
/caros_universalrobot/caros_node/terminate
/caros_universalrobot/caros_serial_device_service_interface/move_lin
/caros_universalrobot/caros_serial_device_service_interface/move_ptp
/caros_universalrobot/caros_serial_device_service_interface/move_ptp_t
/caros_universalrobot/caros_serial_device_service_interface/move_servo_q
/caros_universalrobot/caros_serial_device_service_interface/move_servo_t
/caros_universalrobot/caros_serial_device_service_interface/move_stop
/caros_universalrobot/caros_serial_device_service_interface/move_vel_q
/caros_universalrobot/caros_serial_device_service_interface/move_vel_t
/caros_universalrobot/force_mode_start
/caros_universalrobot/force_mode_stop
/caros_universalrobot/force_mode_update
/caros_universalrobot/get_loggers
/caros_universalrobot/servo_q
/caros_universalrobot/servo_t
/caros_universalrobot/set_io
/caros_universalrobot/set_logger_level
/caros_universalrobot/set_payload
/rosout/get_loggers
/rosout/set_logger_level

charlie@Asgard:~$ rostopic type /caros_universalrobot/caros_serial_device_service_interface/robot_state 
caros_control_msgs/RobotState
charlie@Asgard:~$ rosmsg show caros_control_msgs/RobotState 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
caros_common_msgs/Q q
  float64[] data
caros_common_msgs/Q dq
  float64[] data
bool is_moving
bool is_colliding
bool e_stopped
bool s_stopped
```
