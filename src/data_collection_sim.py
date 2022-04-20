#!/usr/bin/env python3.8
from amiga_group import AmigaMovegroup
from geometry_msgs.msg import Pose, Vector3, TransformStamped, Quaternion
from math import pi, cos , sin
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge.core import CvBridge
import tf2_ros
import rospy
import os
import cv2

class DataCollectionPipeline(AmigaMovegroup):
    def __init__(self) -> None:
        """
        Customly set layer, radius, longitude and latitude for your specific purposes
        """
        super(DataCollectionPipeline, self).__init__()
        self.sphere_layer = 2
        self.sphere_radius = 0.6
        self.longitude = 7
        self.latitude = 8
        self.br = tf2_ros.StaticTransformBroadcaster()
        self.tf_set = []
        self.pose_set = []

    def generate_sphere(self):
        """
        Generate a sphere of view points for the arm to reach for data collection purposes
        """
        # Centre              
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = 'l515_grip_color_optical_frame'
        static_transformStamped.child_frame_id = "point_centre"
        static_transformStamped.transform.translation.x = 0
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 0
        quat = quaternion_from_euler(-0.15785, 0.63107, 0.6549)
        static_transformStamped.transform.rotation.x = -quat[0]
        static_transformStamped.transform.rotation.y = -quat[1]
        static_transformStamped.transform.rotation.z = -quat[2]
        static_transformStamped.transform.rotation.w = -quat[3]
        self.tf_set.append(static_transformStamped)
        
        # Sampling sphere coordinate
        for l in range(self.sphere_layer):
            for i in range(self.longitude):
                for j in range(self.latitude):
                    theta = pi * (90 / (self.latitude + 1)) * ((self.latitude - 1) / 2 - j) / 180
                    phi = pi * (360 / self.longitude / 2) * i / 180
                    x = self.sphere_layer * self.sphere_radius * sin(theta) * cos(phi)
                    y = self.sphere_layer * self.sphere_radius * sin(theta) * sin(phi)
                    z = self.sphere_layer * self.sphere_radius * cos(theta) - 1.2
                    roll = theta * sin(-phi)
                    pitch = theta * cos(phi)
                    yaw = 0
                    frame_label = "point " + str(l) + "," + str(i) + "," + str(j)
                    quat = quaternion_from_euler(roll, pitch, yaw)
                    static_transformStamped = TransformStamped()
                    static_transformStamped.header.stamp = rospy.Time.now()
                    static_transformStamped.header.frame_id = 'l515_grip_color_optical_frame'
                    static_transformStamped.child_frame_id = frame_label
                    static_transformStamped.transform.translation.x = -(x + 0.01757)
                    static_transformStamped.transform.translation.y = -(y - 0.128)
                    static_transformStamped.transform.translation.z = -(z - 0.1425)
                    static_transformStamped.transform.rotation.x = -quat[0]
                    static_transformStamped.transform.rotation.y = -quat[1]
                    static_transformStamped.transform.rotation.z = -quat[2]
                    static_transformStamped.transform.rotation.w = -quat[3]
                    self.pose_set.append(Pose(Vector3(-(x + 0.01757), -(y - 0.128), -(z - 0.1425)), Quaternion(-quat[0], -quat[1], -quat[2], -quat[3])))
                    self.tf_set.append(static_transformStamped)

count = 0
flag = False

def callback_rgb(img):
    if flag == True:
        print("Saved")
        cv2_img = CvBridge.imgmsg_to_cv2(img)
        directory = r'/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/images'
        filename = 'savedImage{0}.jpg'.format(count)
        os.chdir(directory)
        cv2.imwrite(filename, cv2_img)

if __name__ == "__main__":

    rospy.init_node("data_collection_sim", anonymous=True)
    rospy.Subscriber(
            "/l515_grip/color/image_raw",
            Image,
            callback_rgb
            )
    pipeline = DataCollectionPipeline()
    pipeline.generate_sphere()
    pipeline.br.sendTransform(pipeline.tf_set)
    pipeline.set_grasping_home_pose()

    for pose in pipeline.pose_set:
        flag = False
        coord3d = [pose.position.x, pose.position.y, pose.position.z]
        coord3d_rob = pipeline.camera_to_robot(coord3d, 'l515_grip_color_optical_frame', 'base_link')
        target_pose = Pose(coord3d_rob, Quaternion(0, 0, 0, 1))
        pipeline.transform_pose_to_top_pick(target_pose)
        plan = pipeline.plan_pose_goal(pose = target_pose)
        if plan == True:
            print("Succeed")
            count += 1
            flag = True
        
            
    print("{0} our of {1} cases succeeded".format(count, len(pipeline.pose_set)))

    rospy.spin()
