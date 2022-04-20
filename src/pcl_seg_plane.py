import open3d as o3d
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import stats

def crop_filter(cloud, min_x=0, max_x=math.inf, min_y=0, max_y=math.inf, min_z=1, max_z=math.inf):

    points = np.asarray(cloud.points)

    ind = np.where((points[:, 0] > min_x) & (points[:, 0] < max_x) &
                   (points[:, 1] > min_y) & (points[:, 1] < max_y) &
                   (points[:, 2] > min_z) & (points[:, 2] < max_z))[0]

    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    return inlier_cloud, outlier_cloud

pcd = o3d.io.read_point_cloud("/home/yilong/git_ws/src/ur10e_robotiq/amiga_manipulation/data/pointclouds/cropped_05.pcd")
# pcd_np = np.asarray(pcd.points)
# most_common = stats.mode(pcd_np[:, 2])
# print(most_common)
# pcd_histogram = np.histogram(pcd_np[:, 2])
# plt.plot(pcd_histogram)
print(pcd)
in_box_cloud, out_box_cloud = crop_filter(pcd,
                                          min_x=-math.inf, max_x=math.inf,
                                          min_y=-math.inf, max_y=math.inf,
                                          min_z=0.25, max_z=0.52450001)                                   
pcd_np = np.asarray(in_box_cloud.points)
most_common = stats.mode(pcd_np[:, 2])
print(most_common)
print(most_common.mode)
#in_box_cloud.paint_uniform_color([1.0, 0, 0])   # 方框内的点渲染成红色
print(in_box_cloud)   # 位于方框内的点云点的个数
print(out_box_cloud)  # 位于方框外的点云点的个数

o3d.visualization.draw_geometries([in_box_cloud])
# in_box_cloud.paint_uniform_color([1.0, 0, 0])
# print(in_box_cloud)   # 位于方框内的点云点的个数
# print(out_box_cloud)  # 位于方框外的点云点的个数

# o3d.visualization.draw_geometries([in_box_cloud, out_box_cloud])