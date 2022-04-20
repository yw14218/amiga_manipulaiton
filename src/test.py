#!/usr/bin/env python3.6

import rospy
import os
from PIL import Image
from moveit_task_constructor_dexnet.srv import GQCNNGrasp, GQCNNGraspResponse

os.chdir('/home/yilong/catkin_ws/src/deep_grasp_demo/moveit_task_constructor_dexnet/data/images')
rgb="/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/images/rgb_6.png"
depth="/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/images/depth_6.png"
image1 = Image.open(rgb)
image2 = Image.open(depth)
print(image1.format)
print(image1.size)
print(image1.mode)
print(image2.format)
print(image2.size)
print(image2.mode)
# rospy.wait_for_service("/gqcnn_grasp", 10.0)
# proxy = rospy.ServiceProxy("/gqcnn_grasp", GQCNNGrasp)
# result = proxy.call(rgb, depth)
# print(result)