ros_fastsim
===========

ROS interface to the fastsim simulator (simulator for a roomba-like mobile robot) - requires libfastsim

Installation
============
*WARNING*: the current `CMakeList.txt` assumes that `libfastsim` has been compiled and is stored in `$HOME/git/libfastsim`. If your lib is compiled elsewhere, don't forget to update your `CMakeList.txt`.


In your ~/catkin_ws/src:

```
git clone https://github.com/jbmouret/ros_fastsim.git
cd ..
catkin_make
```

Testing
=======
```
roslaunch src/ros_fastsim/envs/fastsim.launch
```

Basic documentation
===================
Please have a look at the libfastsim documentation as well!
http://github.com/jbmouret/libfastsim

ROS parameters (see also the fastsim parameters, which are different)
---------------------------------------------------------------------
-  sync (`<param name="sync" type="bool" value="true"/>`): When sync is true, the robot advance by the value specified by `speed_left` and `speed_right` exactly once, then wait for the next order. When sync is false, the robot continues to use the `same speed_left` and `speed_right` values until it receives a  messages that changes these values.
- settings (`<param name="settings" type="str" value="$(find your_ros_module)/envs/example.xml"/>`): path to the setting file (.xml file) for fastsim (use `$(find your_ros_module)` to use a xml file in your ros module
- path (`<param name="settings" type="str" value="$(find tp_er)/envs/example.xml"/>`): path of the map files (.pgm)


Published topics
-----------------
- `/simu_fastsim/left_bumper` (std_msgs::Bool): left bumper of the mobile robot
- `/simu_fastsim/right_bumper` (std_msgs::Bool): right bumper of the mobile robot
- `/simu_fastsim/lasers` (std_msgs::Float32MultiArray): single lasers (individually added in the xml file of libfastsim)
- `/simu_fastsim/laser_scan` (sensor_msgs::LaserScan): standard laser scanner
- `/simu_fastsim/radars` (std_msgs::Int16MultiArray): radars see "goal" objects. They see through walls if they are configured to do so (see the XML file of fastsim). Each cell of the array corresponds to a radar sensor (typically, one for each goal color), and the number in the cell is the id (an integer) of the activated slice.
- `/simu_fastsim/odom` (nav_msgs::Odometry): odometry

Subscribes to
-------------
- `/simu_fastsim/speed_left`: speed of the left motor (WARNING: see the "sync" ROS parameters)
- `/simu_fastsim/speed_right`: speed of the right motor (WARNING: see the "sync" ROS parameters)

Services
--------
First, import the service messages:
```python
from fastsim.srv import *
```

Two services are available:

- `teleport` (fastsim::Teleport): (float32 x, float32 y, float32 theta) -> (bool ack) : teleport the robot to the specified (x,y,theta) position
- `display` (fastsim::UpdateDisplay): (bool state) -> (bool ack) : activate/de-activate the update of the display (the window is still visible)

Example (Python)
----------------
```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from fastsim.srv import *
from nav_msgs.msg import *
import math
import numpy as np
import random
import copy

class FastSim:
    def __init__(self, name):
        self.node = rospy.init_node(name)
        self.sub_laserscan = rospy.Subscriber("simu_fastsim/laser_scan", 
                                              LaserScan, self._scan_cb)
        self.sub_odom = rospy.Subscriber("simu_fastsim/odom",
                                         Odometry, self._odom_cb)
        self.pub_speed_left = rospy.Publisher("simu_fastsim/speed_left", 
                                              Float32, queue_size=1)
        self.pub_speed_right = rospy.Publisher("simu_fastsim/speed_right",
                                               Float32, queue_size=1)
        self.r = rospy.Rate(200)
        self.laser_scan = []
        self.pos = [0, 0]
        self.speed = 0

    def sleep(self):
        self.r.sleep()

    def reset(self, x, y, theta):
        print 'teleporting to', x,y,theta
        rospy.wait_for_service('simu_fastsim/teleport')
        try:
            teleport = rospy.ServiceProxy('simu_fastsim/teleport', Teleport)
            resp1 = teleport(x, y, theta)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def lasers(self):
        while len(self.laser_scan) == 0: pass
        return self.laser_scan

    def set_speed(self, d1, d2):
        self.pub_speed_left.publish(d1)
        self.pub_speed_right.publish(d2)
    
    def _scan_cb(self, data):
        self.laser_scan = 1 - np.array(data.ranges) / 100.0
    def _odom_cb(self, data):
        self.pos = [data.pose.pose.position.x,
                    data.pose.pose.position.y]
        vx = data.twist.twist.linear.x
        vy = data.twist.twist.linear.y
        self.speed = math.sqrt(vx * vx + vy * vy)
```


