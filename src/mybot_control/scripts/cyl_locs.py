#!/usr/bin/env python
import rospy
import numpy as np
import math

L = []
pi = 3.14159
theta = 0
r = 2.3
z = 0
while z<=2.3 :
    while theta <= 3.2:  #3.2 for error margn to iclude pi  by adding pi/8 
        x = r * math.cos(-theta)
        y = r * math.sin(-theta)
        position = [-x,-y,z]
        rot1 = [0, pi/4, -theta]        
        rot2 = [0, -pi/4, -theta]
        rot3 = [0, 0, -theta]    
        pose1 = np.concatenate((position, rot1), axis=0)
        pose2 = np.concatenate((position, rot2), axis=0)
        pose3 = np.concatenate((position, rot3), axis=0)
        L.append(pose1)
        L.append(pose2)
        L.append(pose3)
        theta+=pi/8
    z+=0.3
    theta = 0
print(L[0:10])
L_np = np.array(L)
np.savetxt("/home/sathish/catkin_ws/src/mybot_ws/src/bot_locations/cyl_locations.txt", L_np, fmt='%1.4e')

