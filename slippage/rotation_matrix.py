# ICP in open3d, only for test
# ICP为精配准

import open3d as o3d
import numpy as np
import copy

import csv


filename = "test02.csv"
csvfile = open(filename, 'w', newline='')
csvwriter = csv.writer(csvfile)

folder = "../5/"

T_overall_fl = np.eye(4)
T_overall_fr = np.eye(4)
T_trans = np.zeros((1,3))

def pc_processing(pc):
    pc = pc.rotate([[1, 0, 0], [0, np.cos(-np.pi/4), -np.sin(-np.pi/4)], [0, np.sin(-np.pi/4), np.cos(-np.pi/4)]])
    pc = pc.rotate([[np.cos(-np.pi/6), 0, np.sin(-np.pi/6)], [0, 1, 0], [-np.sin(-np.pi/6), 0, np.cos(-np.pi/6)]])
    pc = pc.rotate([[np.cos(-np.pi/12), 0, np.sin(-np.pi/12)], [0, 1, 0], [-np.sin(-np.pi/12), 0, np.cos(-np.pi/12)]])
    pc = pc.rotate([[np.cos(np.pi), -np.sin(np.pi), 0], [np.sin(np.pi), np.cos(np.pi), 0], [0, 0, 1]])
    # pc = pc.rotate([[1, 0, 0], [0, np.cos(-np.pi/2), -np.sin(-np.pi/2)], [0, np.sin(-np.pi/2), np.cos(-np.pi/2)]])
    points = np.asarray(pc.points)

    points = points[points[:, 0]>-0.1]
    points = points[points[:, 0]<0.0]
    points = points[points[:, 1]<0.0]
    points = points[points[:, 1]>-0.05]
    points = points[points[:, 2]<0.3]
    points = points[points[:, 2]>0.2]

    pc.points = o3d.utility.Vector3dVector(points)
    # pc, ind = pc.remove_statistical_outlier(10, 0.1)
    return pc

num_frames = 421

if __name__ == "__main__":
    final_pc = o3d.io.read_point_cloud(f"{folder}{num_frames}.ply")
    start_pc = o3d.io.read_point_cloud(f"{folder}{0}.ply")

    for i in range(num_frames):
        # if i % 3 != 0: continue
        # # print(i)
        # i=200

        num_neighbors = 30
        std_ratio = 3.0
        # 读取源点云和目标点云
        # source = o3d.io.read_point_cloud(f"{folder}{i}.ply")
        source = o3d.io.read_point_cloud(f"{folder}{i+1}.ply")
        source = pc_processing(source)
        source, ind = source.remove_statistical_outlier(num_neighbors, std_ratio)
        
        # target = o3d.io.read_point_cloud(f"{folder}{(i+1)}.ply")
        target = o3d.io.read_point_cloud(f"{folder}{(i)}.ply")
        target = pc_processing(target)
        target, ind = target.remove_statistical_outlier(num_neighbors, std_ratio)

        
        if i == 0:
            source_transformed = copy.deepcopy(source)
            test = copy.deepcopy(source)

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
        source_transformed = source_transformed.transform(reg_p2p.transformation)
        final_pc = final_pc.transform(reg_p2p.transformation)

        if i % 10 == 0:
            test.paint_uniform_color([0, 1, 0])
            test2 = copy.deepcopy(source_transformed)
            test2.paint_uniform_color([0, 0, 1])
            # o3d.visualization.draw_geometries([test, test2])
            test = copy.deepcopy(source_transformed)


        T = reg_p2p.transformation

        # 打印变换矩阵
        print("变换矩阵:\n", T)

        csvwriter.writerow([T[0,3],T[1,3],T[2,3],np.sqrt(T[0,3]**2+T[1,3]**2+T[2,3]**2)])



        # print(T)
        T_overall_fl = np.dot(T, T_overall_fl)
        T_overall_fr = np.dot(T_overall_fr, T)
        T_trans = T_trans + T[0:3,3]

    print("变换矩阵:\n", T_overall_fl)
    print("变换矩阵:\n", T_overall_fr)
    print(T_trans)

    # 可视化结果
    # source_transformed.paint_uniform_color([0, 1, 0])
    # target.paint_uniform_color([0, 0, 1])
    # o3d.visualization.draw_geometries([source_transformed, target])
    csvfile.close()
    source.paint_uniform_color([0, 1, 0])
    final_pc.paint_uniform_color([0, 0, 1])
    o3d.visualization.draw_geometries([source, final_pc])
    