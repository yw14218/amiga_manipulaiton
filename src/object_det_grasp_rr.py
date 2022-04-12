from geometry_msgs.msg import Pose, Quaternion
import rospy
import struct
import tf
import numpy as np
from vision_msgs.msg import Detection2D
from sensor_msgs.msg import PointCloud2
from simple_grasp_sim import SimpleGraspPipeline
from std_srvs.srv import Empty

datatype = {1:1, 2:1, 3:2, 4:2, 5:4, 6:4, 7:4, 8:8}

class ObjDetGraspPipeline(SimpleGraspPipeline):
    def __init__(self) -> None:
        super(ObjDetGraspPipeline, self).__init__()

        rospy.init_node("grasp_rr", anonymous=True)
        rospy.Subscriber("obj_detect/detectnet/detections", Detection2D, self.callback_detnet)

        self.xCenterObject = None
        self.yCenterObject = None

    def callback_detnet(self, data):

        for detection in data.detections:
            if detection.results.id == '44':
                self.xCenterObject = int(detection.bbox.center.x)
                self.yCenterObject = int(detection.bbox.center.y)
                print("center_X: " + str(self.xCenterObject) + " center_Y: " + str(self.yCenterObject) )
                # rospy.loginfo(
                #     box.Class + ": " + 
                #     "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(
                #         box.xmin, box.xmax, box.ymin, box.ymax
                #     )
                # )

    @staticmethod
    def get_xyz(point_2d, pc_msg):
            arrayPosition = point_2d[0]*pc_msg.row_step + point_2d[1]*pc_msg.point_step # point_2d: y,x
            pos_x = arrayPosition + pc_msg.fields[0].offset # X has an offset of 0
            len_x = datatype[pc_msg.fields[0].datatype]
            pos_y = arrayPosition + pc_msg.fields[1].offset # Y has an offset of 4
            len_y = datatype[pc_msg.fields[1].datatype]
            pos_z = arrayPosition + pc_msg.fields[2].offset # Z has an offset of 8
            len_z = datatype[pc_msg.fields[2].datatype]

            try:
                x = struct.unpack('f', pc_msg.data[pos_x: pos_x+len_x])[0] # read 4 bytes as a float number
                y = struct.unpack('f', pc_msg.data[pos_y: pos_y+len_y])[0]
                z = struct.unpack('f', pc_msg.data[pos_z: pos_z+len_z])[0]
                return [x,y,z]
            except:
                return None

if __name__ == "__main__":

    rospy.wait_for_service("/amiga_gripper/init_gripper", 10.0)
    rospy.wait_for_service("/amiga_gripper/open_gripper", 10.0)
    rospy.wait_for_service("/amiga_gripper/close_gripper", 10.0)
    gripper_init_srv = rospy.ServiceProxy("/amiga_gripper/init_gripper", Empty)
    gripper_open_srv = rospy.ServiceProxy("/amiga_gripper/open_gripper", Empty)
    gripper_close_srv = rospy.ServiceProxy("/amiga_gripper/close_gripper", Empty)

    print(gripper_init_srv)

    init = gripper_init_srv()
    close = gripper_close_srv()
    open = gripper_open_srv()
    
    gripper_init_srv.call()
    gripper_close_srv.call()
    gripper_open_srv.call()
    print("hello")


    pipeline = ObjDetGraspPipeline()
    pipeline.object = "bottle" 

    if pipeline.xCenterObject is not None: 
        detected_xCenterObject= pipeline.xCenterObject
        detected_yCenterObject= pipeline.yCenterObject
        pc_msg = rospy.wait_for_message('/l515_grip/depth/color/points', PointCloud2)
        detected_3dObject = pipeline.get_xyz([detected_xCenterObject, detected_yCenterObject], pc_msg)
        if detected_3dObject is not None:
            detected_3dObjectRobotFrame = pipeline.camera_to_robot(detected_3dObject, 'l515_grip_color_optical_frame', 'base_link')
            target_pose = Pose(detected_3dObject, Quaternion(0, 0, 0, 1))
            pipeline.transform_pose_to_top_pick(target_pose)
            plan = pipeline.plan_pose_goal(pose = target_pose)
            gripper_close_srv.call()
            pipeline.go_up()

    rospy.spin()
