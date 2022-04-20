#!/usr/bin/env python3.8
import rospy
import os
import time

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2, Image
from simple_grasp_sim import SimpleGraspPipeline
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Quaternion, TransformStamped

os.environ['ROS_IP'] = '10.0.3.12'

id_dict = {
    'bottle' : 44,
    'plate' : 45,
    'wine glass' : 46,
    'cup' : 47,
    'fork' : 48,
    'knife' : 49,
    'spoon' : 50,
    'bowl' : 51,
    'banana' : 52,
    'apple' : 53,
    'sandwich' : 54,
    'orange' : 55,
    'broccoli' : 56,
    'carrot' : 57,
    'hot dog' : 58,
    'pizza' : 59,
    'donut' : 60,
    'cake' : 61,
    'blender' : 83,
    'teddy bear' : 88
}

class ObjDetGraspPipeline(SimpleGraspPipeline):
    def __init__(self) -> None:
        super(ObjDetGraspPipeline, self).__init__()

        rospy.init_node("grasp_rr", anonymous=True)
        rospy.Subscriber("obj_detect/detectnet/detections", Detection2DArray, self.callback_detnet)
        rospy.Subscriber('/l515_grip/depth/color/points', PointCloud2, self.callback_pc)
        # rospy.Subscriber('obj_detect/detectnet/overlay', Image, self.callback_obj) # shape : (720, 1080)
        # rospy.Subscriber('l515_grip/color/image_raw', Image, self.callback_img) # shape : (720, 1080)

        rospy.wait_for_service("/amiga_gripper/init_gripper", 10.0)
        rospy.wait_for_service("/amiga_gripper/open_gripper", 10.0)
        rospy.wait_for_service("/amiga_gripper/close_gripper", 10.0)
        self.gripper_init = rospy.ServiceProxy("/amiga_gripper/init_gripper", Empty)
        self.gripper_open = rospy.ServiceProxy("/amiga_gripper/open_gripper", Empty)
        self.gripper_close = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)

        self.gripper_init()
        self.xCenterObject = None
        self.yCenterObject = None
        self.detected_3dObject = None
        
    def callback_detnet(self, data):
        for detection in data.detections:
            if detection.results[0].id == id_dict[self.object]:
                self.xCenterObject = int(detection.bbox.center.x)
                self.yCenterObject = int(detection.bbox.center.y)
                print("center_X: " + str(self.xCenterObject) + " center_Y: " + str(self.yCenterObject))

    def callback_pc(self, data):
        # m = ros_numpy.numpify(data) shape : (720, 1280)
        if self.xCenterObject is not None and self.yCenterObject is not None: 
            self.detected_3dObject = self.get_xyz([self.yCenterObject, self.xCenterObject], data)
            if self.detected_3dObject is not None:
                print("------------------------------------INFO-------------------------------------------")
                print(self.detected_3dObject)
                detected_3dObjectRobotFrame = pipeline.camera_to_robot(self.detected_3dObject, 'l515_grip_color_optical_frame', 'base_link')
                target_pose = Pose(detected_3dObjectRobotFrame, Quaternion(0, 0, 0, 1))
                target_pose = self.transform_pose_to_top_pick(target_pose)
                target_pose.position.z += 0.15 # gripper palm offset
                print(target_pose)

                pose_in_list = [target_pose.position.x, target_pose.position.y, target_pose.position.z, 
                    target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w]
                self.static_tf_broadcast('base_link', 'teddy_bear_grasp_link', pose_in_list)

                plan = self.plan_pose_goal(pose = target_pose)
                #self.display_trajectory(plan)
                self.gripper_close()
                self.go_up()
                time.sleep(5)
                rospy.signal_shutdown("Pipeline ended. Terminated.")

if __name__ == "__main__":

    pipeline = ObjDetGraspPipeline()
    pipeline.object = 'teddy bear' 
    pipeline.gripper_open()

    rospy.spin()
