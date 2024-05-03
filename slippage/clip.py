import os
import numpy as np
import open3d as o3d


def pc_processing(pc):
    points = np.asarray(pc.points)

    # #extract along the x coordinate
    # points = points[points[:, 0]<0.26901]
    # points = points[points[:, 0]>0.078136]
    # # points = points[points[:, 1]<0.1]

    # #extract along the y coordinate
    # points = points[points[:, 1]<0.103521]
    # points = points[points[:, 1]>-0.109386]

    # #extract along the z coordinate
    # points = points[points[:, 2]<0.283319]
    # points = points[points[:, 2]>0.248344]


    # #第二次尝试
    #   #extract along the x coordinate
    # points = points[points[:, 0]<0.10]
    # points = points[points[:, 0]>-0.10]
    # # points = points[points[:, 1]<0.1]

    # #extract along the y coordinate
    # points = points[points[:, 1]<0.113521]
    # points = points[points[:, 1]>-0.119386]

    # #extract along the z coordinate
    # points = points[points[:, 2]<0.283319]
    # points = points[points[:, 2]>0.248344]

    #hlt suggestion
    #extract along the x coordinate
    points = points[points[:, 0]<0.06]
    points = points[points[:, 0]>-0.03]
    # points = points[points[:, 1]<0.1]

    #extract along the y coordinate
    points = points[points[:, 1]<0.05]
    points = points[points[:, 1]>-0.05]

    #extract along the z coordinate
    points = points[points[:, 2]<0.3]
    # points = points[points[:, 2]>0.248344]




    pc.points = o3d.utility.Vector3dVector(points)
    return pc

# 获取当前文件夹路径
current_directory = os.getcwd()

# 读取PLY点云文件
point_cloud = o3d.io.read_point_cloud(os.path.join(current_directory, "pointcloud_raw/0.ply"))

# 处理点云
processed_point_cloud = pc_processing(point_cloud)

# 导出处理后的PLY文件
o3d.io.write_point_cloud(os.path.join(current_directory, "pointcloud_clip/0.ply"), processed_point_cloud)