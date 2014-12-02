ros_fastsim
===========

ROS interface to the fastsim simulator (simulator for a roomba-like mobile robot) - requires libfastsim

Installation
============

In your ~/catkin_ws/src:

```
git clone https://github.com/jbmouret/ros_fastsim.git
catkin_make
```

Testing
=======
```
roslaunch src/ros_fastsim/envs/fastsim.launch
```

Basic documentation
===================

ROS parameters (see also the fastsim parameters, which are different)
---------------------------------------------------------------------
-  sync (`<param name="sync" type="bool" value="true"/>`): When sync is true, the robot advance by the value specified by `speed_left` and `speed_right` exactly once, then wait for the next order. When sync is false, the robot continues to use the `same speed_left` and `speed_right` values until it receives a  messages that changes these values.
- settings (`<param name="settings" type="str" value="$(find your_ros_module)/envs/example.xml"/>`): path to the setting file (.xml file) for fastsim (use `$(find your_ros_module)` to use a xml file in your ros module
- path (`<param name="settings" type="str" value="$(find tp_er)/envs/example.xml"/>`): path of the map files (.pgm)


Published topics:
-----------------
- `/simu_fastsim/left_bumper` (std_msgs::Bool): left bumper of the mobile robot
- `/simu_fastsim/right_bumper` (std_msgs::Bool): right bumper of the mobile robot
- `/simu_fastsim/lasers` (std_msgs::Float32MultiArray): single lasers (individually added in the xml file of libfastsim)
- `/simu_fastsim/laser_scan` (sensor_msgs::LaserScan): standard laser scanner
- `/simu_fastsim/radars` (std_msgs::Int16MultiArray): radars see "goal" objects; they see through walls
- `/simu_fastsim/odom` (nav_msgs::Odometry): odometry

Subscribes to:
-------------
- `/simu_fastsim/speed_left`: speed of the left motor (WARNING: see the "sync" ROS parameters)
- `/simu_fastsim/speed_right`: speed of the right motor (WARNING: see the "sync" ROS parameters)

Services
--------
- `teleport` (fastsim::Teleport): (float32 x, float32 y, float32 theta) -> (bool ack) : teleport the robot to the specified (x,y,theta) position
- `display` (fastsim::UpdateDisplay): (bool state) -> (bool ack) : activate/de-activate the update of the display (the window is still visible)

