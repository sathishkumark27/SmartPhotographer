#!/usr/bin/env python
import sys
import rospy
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import numpy as np

datafilepath = "/home/sathish/catkin_ws/src/rpg_ig_active_reconstruction/example/flying_gazebo_stereo_cam/config/test.txt"
locs  = np.loadtxt(datafilepath, delimiter='   ', skiprows=1)

L = []
for p in locs:
    q = p[3:]
    e = euler_from_quaternion(q)
    pose = np.concatenate((p[0:3], e), axis=0)
    L.append(pose)
print(L[0:10])
L_np = np.array(L)
np.savetxt("/home/sathish/catkin_ws/src/mybot_ws/src/bot_locations/locations.txt", L_np)