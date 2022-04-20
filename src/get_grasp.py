#!/usr/bin/env python3.8
import rospy
import actionlib
import os
import time
import open3d as o3d

from moveit_task_constructor_msgs.msg import SampleGraspPosesAction, SampleGraspPosesGoal, SampleGraspPosesActionFeedback
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2, Image
from simple_grasp_sim import SimpleGraspPipeline
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Quaternion, TransformStamped
from moveit_task_constructor_gpd.srv import PointCloud
from cloud_processing import process_cloud

#os.environ['ROS_IP'] = '10.0.3.12'

class GPDGraspPipeline():
    def __init__(self) -> None:
        super(GPDGraspPipeline, self).__init__()

        rospy.init_node("gpd_grasp_rr", anonymous=True)
        rospy.Subscriber("/generate_grasps/feedback", SampleGraspPosesActionFeedback, self.callback_gpd)
        rospy.wait_for_service("/save_point_cloud", 10.0)
        #rospy.wait_for_service("/amiga_gripper/init_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/init_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/open_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/close_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/pinch_mode_gripper", 10.0)
        # rospy.wait_for_service("/amiga_gripper/basic_mode_gripper", 10.0)
        self.collect_pc = rospy.ServiceProxy("/save_point_cloud", PointCloud)
        # self.gripper_init = rospy.ServiceProxy("/amiga_gripper/init_gripper", Empty)
        # self.gripper_open = rospy.ServiceProxy("/amiga_gripper/open_gripper", Empty)
        # self.gripper_close = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        # self.gripper_set_pinch_mode = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)
        # self.gripper_set_basic_mode = rospy.ServiceProxy("/amiga_gripper/basic_gripper", Empty)

        # self.gripper_init()
        self.client = actionlib.SimpleActionClient('generate_grasps', SampleGraspPosesAction)
        self.grasp_pose = None
        self.trial = 0

    def callback_gpd(self, data):
        self.grasp_pose = data.feedback.grasp_candidates[0].pose
        print(self.grasp_pose)

    def grasp_detection(self):
        self.client.wait_for_server()
        goal = SampleGraspPosesGoal()
        self.client.send_goal(goal)
        time.sleep(5)
        self.client.wait_for_result()
        return self.client.get_result()

if __name__ == "__main__":

    pipeline = GPDGraspPipeline()
    while True:
        os.chdir('/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/tmp')
        InPath = "tmp.pcd"
        OutPath = "tmp_processed.pcd"
        pipeline.collect_pc("tmp.pcd")
        time.sleep(5)
        processed_cloud = process_cloud(InPath)
        o3d.io.write_point_cloud(OutPath, processed_cloud)
        print("save succeed")
        result = pipeline.grasp_detection()
        print(result.grasp_state)
        os.remove(InPath)
        os.remove(OutPath)
        pipeline.trial += 1
        


