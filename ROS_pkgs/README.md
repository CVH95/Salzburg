# Export ROS MASTER URI and IP

We have to export the URI and the IP for every terminal in order to be able to use the camera. Since it is quite annoying use the shell script in /ROS\_pkgs:

```sh
$ . ROS_pkgs/export_master.sh
```

Use `.` instead of `./` to execute it to tell the system that you want these changes to be applied in the current terminal.


