# GICP in open3d, only for test
# GICP为粗配准/全局配准
# Generalized Iterative Closest Point

import open3d as o3d
import numpy as np
import csv

# 保存为 csv

folder = "../5/"

if __name__ == "__main__":

    source = o3d.io.read_point_cloud(f"{folder}{200}.ply")

    # 定义旋转矩阵
    rotation_matrix = np.array([[ 0.31680028, -0.6073299,   0.72855197],
                                [-0.14842866,  0.72690855,  0.67050197],
                                [-0.93680655, -0.32055321,  0.14013967]])
    


    # rotation_matrix = np.array([[ 0.83899053  0.47210305 -0.2705801 ],
    #                             [-0.2213919   0.7503976   0.62280741],
    #                             [ 0.49707194 -0.46262528  0.73409627]])

    # 将点云转换为 NumPy 数组
    points = np.asarray(source.points)

    # 将点云绕原点旋转
    rotated_points = np.dot(points, rotation_matrix.T)  # 注意需要转置矩阵

    # 创建新的点云对象并设置旋转后的点
    rotated_pcd = o3d.geometry.PointCloud()
    rotated_pcd.points = o3d.utility.Vector3dVector(rotated_points)

    # 可视化旋转后的点云
    o3d.visualization.draw_geometries([rotated_pcd])