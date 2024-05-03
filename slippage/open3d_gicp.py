# GICP in open3d, only for test
# GICP为粗配准/全局配准
# Generalized Iterative Closest Point

import open3d as o3d
import numpy as np

folder = "./pointcloud_raw/"

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
    for i in range(55):
    # for i in range(1):
        # 读取源点云和目标点云
        source = o3d.io.read_point_cloud(f"{folder}{i}.ply")
        # source = pc_processing(source)
        target = o3d.io.read_point_cloud(f"{folder}{(i+1)}.ply")
        # target = pc_processing(target)

        if i == 0:
            origin_pc = source
            source_transformed = source
        
        # 设置GICP参数
        trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])  # 设置初始的变换矩阵

        # 运行GICP算法
        reg_p2p = o3d.pipelines.registration.registration_generalized_icp(
            source, target, max_correspondence_distance=0.05, init=trans_init,
            estimation_method=o3d.pipelines.registration.TransformationEstimationForGeneralizedICP())  

        # 将变换后的点云应用于源点云
        source_transformed = source_transformed.transform(reg_p2p.transformation)

        T = reg_p2p.transformation

        # 打印变换矩阵
        # print("变换矩阵:\n", T)


        # print(T)
        T_overall_fl = np.dot(T, T_overall_fl)
        T_overall_fr = np.dot(T_overall_fr, T)

    print("变换矩阵:\n", T_overall_fl)
    print("变换矩阵:\n", T_overall_fr)


    ##### ICP start

    threshold = 0.01  # 设置一个阈值，用于剔除大于该阈值的配准误差点
    
    # 运行ICP算法
    reg_p2p = o3d.pipelines.registration.registration_icp(
        origin_pc, target, threshold, T_overall_fl,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())

    # 将变换后的点云应用于源点云
    source_transformed = origin_pc.transform(reg_p2p.transformation)

    T = reg_p2p.transformation

    print("变换矩阵 after ICP:\n", T)

    #### ICP end


    # 可视化结果
    source_transformed.paint_uniform_color([0, 1, 0])
    target.paint_uniform_color([0, 0, 1])
    o3d.visualization.draw_geometries([source_transformed, target])