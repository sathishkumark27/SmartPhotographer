#!/usr/bin/env python
import sys
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
#from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import numpy as np

class get_image_and_pose : 
    def __init__(self):
        self.bridge = CvBridge()
        self.p_x = 0
        self.p_y = 0
        self.p_z = 0
        self.q_x = 0
        self.q_y = 0
        self.q_z = 0
        self.q_w = 0
        self.raw_image = None
        self.dir = "/home/sathish/catkin_ws/src/mybot_ws/src/images/"
        self.count = 0

    def callbackpose(self, obj):
        #if (obj.name == "myrobot"):
        self.p_x = obj.pose[2].position.x
        self.p_y = obj.pose[2].position.y
        self.p_z = obj.pose[2].position.z
        self.q_x = obj.pose[2].orientation.x
        self.q_y = obj.pose[2].orientation.y
        self.q_z = obj.pose[2].orientation.z
        self.q_w = obj.pose[2].orientation.w
        rospy.loginfo(rospy.get_caller_id() + ' Model Name %s Pose %s \n', obj.name[2], obj.pose[2])

    def callbackimage(self, obj):  
        #rospy.loginfo(rospy.get_caller_id() + ' Model Name %s \n', obj)      
        try:
            self.raw_image = self.bridge.imgmsg_to_cv2(obj, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        #if self.raw_image != None:
        #self.save_image()
        

    def save_image(self):
        imname = str(self.p_x)+'_'+str(self.p_y)+'_'+str(self.p_z)+'_'+str(self.q_x)+'_'+str(self.q_y)+'_'+str(self.q_z)+'_'+str(self.q_w)+'_'
        #cv2.imshow("ros2cv image", self.raw_image)
        path = self.dir + imname + str(self.count)+ ".jpg"
        self.count+=1
        cv2.imwrite(path, self.raw_image)
        if (" " == cv2.waitKey(3)):
            return

    def save_image_pose(self, pose, euler):
        #imname = str(pose.position.x)+'_'+str(pose.position.y)+'_'+str(pose.position.z)+'_'+str(pose.orientation.x)+'_'+str(pose.orientation.y)+'_'+str(pose.orientation.z)+'_'+str(pose.orientation.w)+'_'
        #cv2.imshow("ros2cv image", self.raw_image)
        imname = str(pose.position.x)+'_'+str(pose.position.y)+'_'+str(pose.position.z)+'_'+str(euler[0])+'_'+str(euler[1])+'_'+str(euler[2])+'_'
        path = self.dir + imname + str(self.count)+ ".jpg"
        self.count+=1
        cv2.imwrite(path, self.raw_image)
        if (" " == cv2.waitKey(3)):
            return


    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('bot_pose_rawimage_listener', anonymous=True)
        rate = rospy.Rate(0.1) # 10hz
        #rospy.Subscriber('/gazebo/model_states', ModelStates, self.callbackpose)            
        rospy.Subscriber('/mybot/camera1/image_raw', Image, self.callbackimage)  

        # Subscribe to the state(position) of the ball
        mypause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        myunpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        set_pos = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        #datafilepath = "/home/sathish/catkin_ws/src/rpg_ig_active_reconstruction/example/flying_gazebo_stereo_cam/config/test.txt"
        #locs  = np.loadtxt(datafilepath, delimiter='   ',  skiprows=1)
        cyl_loc_path = "/home/sathish/catkin_ws/src/mybot_ws/src/bot_locations/cyl_locations.txt"
        locs  = np.loadtxt(cyl_loc_path, delimiter=' ',  skiprows=0)
        print(locs[0:10])

        #q = quaternion_from_euler(0, 0, 1.5707)
        #print(q)

        # Set pose of the model
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = -3.0
        pose.position.z = 0.6
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.707108079859
        pose.orientation.w = 0.707105482511

        # Set robot ought to be moved
        state = ModelState() # for service it should be ModelState 
        state.model_name = "myrobot"
        state.pose = pose
        state.reference_frame = 'world'
        try:
            return_msg = set_pos(state)
            print return_msg.status_message
        except Exception, e:
            rospy.logerr('Error on Calling Service: %s', str(e))

        #self.save_image_pose(pose)
        #euler poses roll(x-axis), pitch (y-axis) and Yaw (z-axis) in radians
        z = 0.5
        x = y = 2.3
        positions = [[-x, 1, z], [0, y, z], [x, 0, z], [0, -y, z],  [-x, 1, z], [0, y, z], [x, 0, z], [0, -y, z]]
        piby2 = 1.5707
        euler_poses = [[0,0,0], [0,0,-piby2], [0,0,2*piby2], [0,0,piby2], [0,0,0], [0,0,-piby2], [0,0,2*piby2], [0,0,piby2]]
                
        while not rospy.is_shutdown():
            distance = 0
            for p in locs :
            #for p,o in zip(positions, euler_poses):
                #Pause physics
                try:
                    mypause()
                except Exception, e:
                    rospy.logerr('Error on Calling Service: %s', str(e))

                q = quaternion_from_euler(p[3], p[4], p[5]) #rotate 90 wrt Z for each potional movement
                #q = p[3:7]
                #e = euler_from_quaternion(q)
                e = p[3:]
                #Renew the position of the model
                pose.position.x = p[0]
                pose.position.y = p[1]
                pose.position.z = p[2]
                pose.orientation.x = q[0]#p[3]
                pose.orientation.y = q[1]#p[4]
                pose.orientation.z = q[2]#p[5]
                pose.orientation.w = q[3]#p[6]
                state.pose = pose


                # Call the Service to publish position info
                #rospy.wait_for_service('/gazebo/set_model_state')
                try:
                    return_msg = set_pos(state)
                    print return_msg.status_message
                except Exception, e:
                    rospy.logerr('Error on Calling Service: %s', str(e))

                #rate.sleep()
                print("pose = ", p[0:3])
                print("e = ", e)
                #print("o = ", o)
                print("q = ", q)

                #self.save_image_pose(pose)

                # print pose
                # filename = './parallelstereo/r_%s.jpg' % distance 
                # cv2.imwrite(filename, ic1.raw_image)
                # filename = './parallelstereo/l_%s.jpg' % distance
                # cv2.imwrite(filename, ic2.raw_image)

                # Resume physics
                #self.save_image_pose(pose)
                try:
                    myunpause()
                except Exception, e:
                    rospy.logerr('Error on Calling Service: %s', str(e))
                #self.save_image_pose(pose)
                #sub = rospy.Subscriber('/mybot/camera1/image_raw', Image, self.callbackimage)
                rate.sleep()
                self.save_image_pose(pose, e)
                #sub.unregister()
            break

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        cv2.destroyAllWindows()

imagepose = get_image_and_pose()
imagepose.listener()

