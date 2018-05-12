# Testing Collision detector

Seems to detect collisions now. Tested with the following setups:


### Collision == TRUE

![alt text](collision_1_08.05.2018.png)


 - On `collision_detector` node:

```sh
charlie@Asgard:~/catkin_ws$ rosrun planner collision_detector 
	>> WorkCell: /home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml
	>> Found device: UR1
	>> Red Ball created and added succesfully to the WorkCell.
	>> Created in: Transform3D(Vector3D(0, 0, 0), Rotation3D(0, 0, 0, 0, 0, 0, 0, 0, 0))
Vector3D(-232.8, -391.867, 101.732)
	>> Transform3D of 0x1c8a2b0:
	>> Transform3D(Vector3D(-232.8, -391.867, 101.732), Rotation3D(1, 0, 0, 0, 1, 0, -0, 0, 1))

Vector3D(-88.1476, -412.237, 323.352)
	>> RedBall:   Transform3D(Vector3D(-88.1476, -412.237, 323.352), Rotation3D(1, 0, 0, 0, 1, 0, -0, 0, 1))
Configuration in collision: Q[6]{1.21745, -1.76393, -1.87246, -1.55601, 3.5312, 0.122671}
Colliding frames: 
UR1.Joint3 UR1.Joint6
UR1.Joint4 UR1.Joint6
Collision detected in Q = Q[6]{1.21745, -1.76393, -1.87246, -1.55601, 3.5312, 0.122671}
	>> Path checked in 0.214 seconds.

Vector3D(-88.6684, -411.523, 324.278)
	>> RedBall:   Transform3D(Vector3D(-88.6684, -411.523, 324.278), Rotation3D(1, 0, 0, 0, 1, 0, -0, 0, 1))
Configuration in collision: Q[6]{1.21745, -1.76393, -1.87246, -1.55601, 3.5312, 0.122671}
Colliding frames: 
UR1.Joint3 UR1.Joint6
UR1.Joint4 UR1.Joint6
Collision detected in Q = Q[6]{1.21745, -1.76393, -1.87246, -1.55601, 3.5312, 0.122671}
	>> Path checked in 0.139 seconds.

Vector3D(-88.1476, -412.237, 323.352)
	>> RedBall:   Transform3D(Vector3D(-88.1476, -412.237, 323.352), Rotation3D(1, 0, 0, 0, 1, 0, -0, 0, 1))
Configuration in collision: Q[6]{1.21745, -1.76393, -1.87246, -1.55601, 3.5312, 0.122671}
Colliding frames: 
UR1.Joint3 UR1.Joint6
UR1.Joint4 UR1.Joint6
Collision detected in Q = Q[6]{1.21745, -1.76393, -1.87246, -1.55601, 3.5312, 0.122671}
	>> Path checked in 0.137 seconds.

Vector3D(-88.6684, -411.523, 324.278)
	>> RedBall:   Transform3D(Vector3D(-88.6684, -411.523, 324.278), Rotation3D(1, 0, 0, 0, 1, 0, -0, 0, 1))
Configuration in collision: Q[6]{1.21745, -1.76393, -1.87246, -1.55601, 3.5312, 0.122671}
Colliding frames: 
UR1.Joint3 UR1.Joint6
UR1.Joint4 UR1.Joint6
Collision detected in Q = Q[6]{1.21745, -1.76393, -1.87246, -1.55601, 3.5312, 0.122671}
	>> Path checked in 0.138 seconds.
```

 - On `collision_listener` node:

```sh
charlie@Asgard:~/catkin_ws$ rosrun planner collion_listener 
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
Found Collisions?   
YES
```

### Collision == FALSE

![alt text]()

## Changed add\_red\_ball() function.

Added Model3D and Geometry to the RigidObject. This should create the actual object and give collisions.

----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Checked on test\_collision.cpp

NOW WORKING!!!!! (09-05-2018)

```
charlie@Asgard:~/catkin_ws$ rosrun planner test_collision 

	>> WorkCell: /home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2_with_ball/WC2_Scene.wc.xml
	>> Found device: UR1
	>> RedBall in:   Transform3D(Vector3D(0, 0, 0), Rotation3D(1, 0, 0, 0, 1, 0, -0, 0, 1))
	>> Red Ball is in the WorkCell.
	>> Created in: Transform3D(Vector3D(0, 0, 0), Rotation3D(1, 0, 0, 0, 1, 0, -0, 0, 1))

	>> Frame RedBall moved to:
	>> Transform3D(Vector3D(0.36, -0.9, 1.5), Rotation3D(1, 0, 0, 0, 1, 0, -0, 0, 1))

	>> CurrentQ = Q[6]{-0.087, -1.007, 0, -1.573, 0, 0}
Configuration in collision: Q[6]{-0.087, -1.007, 0, -1.573, 0, 0}
Colliding frames: 
RedBall UR1.Joint3
RedBall UR1.Joint4
	>> Collision detected in Q = Q[6]{-0.087, -1.007, 0, -1.573, 0, 0}
charlie@Asgard:~/catkin_ws$ 
```



Richard: added on 11 May

	>> RedBall:   Transform3D(Vector3D(0.703458, -0.443006, 1.25935), Rotation3D(6.12323e-17, -1, 0, 1, 6.12323e-17, 0, 0, 0, 1))
	>> Obstacle found
Configuration in collision: Q[6]{-1.09953, -1.14467, -2.10252, -0.682953, 1.46606, -0.300364}
Colliding frames: 
RedBall UR1.Joint5
Collision detected in Q = Q[6]{-1.09953, -1.14467, -2.10252, -0.682953, 1.46606, -0.300364}
	>> Path checked in 0.171 seconds.

	>> RedBall:   Transform3D(Vector3D(0.386753, -0.679095, 1.28039), Rotation3D(6.12323e-17, -1, 0, 1, 6.12323e-17, 0, 0, 0, 1))
	>> Obstacle found
Configuration in collision: Q[6]{-0.425416, -0.722257, -1.41736, 0.170045, 1.24767, 0.423711}
Colliding frames: 
RedBall UR1.Joint2
Collision detected in Q = Q[6]{-0.425416, -0.722257, -1.41736, 0.170045, 1.24767, 0.423711}
	>> Path checked in 0.178 seconds.

	>> RedBall:   Transform3D(Vector3D(-nan, -nan, -nan), Rotation3D(6.12323e-17, -1, 0, 1, 6.12323e-17, 0, 0, 0, 1))
	>> Obstacle found
	>> Path checked in 0.202 seconds.
