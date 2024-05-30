# ICP in open3d, only for test
# ICP为精配准

import open3d as o3d
import numpy as np
import csv

filename = "test02.csv"
csvfile = open(filename, 'w', newline='')
csvwriter = csv.writer(csvfile)

folder = "./4/"

T_overall_fl = np.eye(4)
T_overall_fr = np.eye(4)


def pc_processing(pc):
    points = np.asarray(pc.points)
    points = points[points[:, 0]>-0.05]
    points = points[points[:, 0]<0.3]
    # points = points[points[:, 1]<0.1]
    points = points[points[:, 2]<0.73]
    pc.points = o3d.utility.Vector3dVector(points)
    return pc

if __name__ == "__main__":
    for i in range(79):
        # 读取源点云和目标点云
        source = o3d.io.read_point_cloud(f"{folder}{i}.ply")
        source = pc_processing(source)
        target = o3d.io.read_point_cloud(f"{folder}{(i+1)}.ply")
        target = pc_processing(target)
        
        # 设置ICP参数
        threshold = 0.003  # 设置一个阈值，用于剔除大于该阈值的配准误差点
        trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0017],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])  # 设置初始的变换矩阵

        # 运行ICP算法
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())

        # 将变换后的点云应用于源点云
        source_transformed = source.transform(reg_p2p.transformation)

        T = reg_p2p.transformation

        # 打印变换矩阵
        # print("变换矩阵:\n", T)


        # print(T)
        T_overall_fl = np.dot(T, T_overall_fl)
        T_overall_fr = np.dot(T_overall_fr, T)

    print("变换矩阵:\n", T_overall_fl)
    print("变换矩阵:\n", T_overall_fr)

    # 可视化结果
    # o3d.visualization.draw_geometries([source_transformed, target])