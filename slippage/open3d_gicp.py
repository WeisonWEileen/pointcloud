# GICP in open3d, only for test
# GICP为粗配准/全局配准
# Generalized Iterative Closest Point

import open3d as o3d
import numpy as np
import csv

# 保存为 csv

folder = "../5/"

filename = "test03.csv"
csvfile = open(filename, 'w', newline='')
csvwriter = csv.writer(csvfile)

T_overall_fl = np.eye(4)
T_overall_fr = np.eye(4)

pc_frame_num = 20



def pc_processing(pc):
    pc = pc.rotate([[1, 0, 0], [0, np.cos(-np.pi/4), -np.sin(-np.pi/4)], [0, np.sin(-np.pi/4), np.cos(-np.pi/4)]])
    pc = pc.rotate([[np.cos(-np.pi/6), 0, np.sin(-np.pi/6)], [0, 1, 0], [-np.sin(-np.pi/6), 0, np.cos(-np.pi/6)]])
    pc = pc.rotate([[np.cos(-np.pi/12), 0, np.sin(-np.pi/12)], [0, 1, 0], [-np.sin(-np.pi/12), 0, np.cos(-np.pi/12)]])
    pc = pc.rotate([[np.cos(np.pi), -np.sin(np.pi), 0], [np.sin(np.pi), np.cos(np.pi), 0], [0, 0, 1]])
    # pc = pc.rotate([[1, 0, 0], [0, np.cos(-np.pi/2), -np.sin(-np.pi/2)], [0, np.sin(-np.pi/2), np.cos(-np.pi/2)]])
    points = np.asarray(pc.points)
    # points = points[points[:, 0]>-0.03]
    # points = points[points[:, 0]<0.06]
    # points = points[points[:, 1]<0.05]
    # points = points[points[:, 1]>-0.01]
    # points = points[points[:, 2]<0.3]

    points = points[points[:, 0]>-0.1]
    points = points[points[:, 0]<0.0]
    points = points[points[:, 1]<0.0]
    points = points[points[:, 1]>-0.05]
    points = points[points[:, 2]<0.3]
    points = points[points[:, 2]>0.2]

    # points = points[points[:, 0]>0]
    # points = points[points[:, 0]<0.04]
    # points = points[points[:, 1]<0.05]
    # points = points[points[:, 1]>-0.01]
    # points = points[points[:, 2]<0.263]

    pc.points = o3d.utility.Vector3dVector(points)
    return pc

if __name__ == "__main__":
    
    ttt = pc_processing(o3d.io.read_point_cloud(f"{folder}{120}.ply"))

    for i in range(120,150):
    # for i in range(1):
        # 读取源点云和目标点云
        source = o3d.io.read_point_cloud(f"{folder}{i}.ply")
        source = pc_processing(source)
        target = o3d.io.read_point_cloud(f"{folder}{(i+1)}.ply")
        target = pc_processing(target)

        if i == 120:
            origin_pc = source
            source_transformed = source
        
        # 设置GICP参数
        trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])  # 设置初始的变换矩阵
        # 一帧粗匹配，然后一帧精匹配

        # 运行GICP算法, 对应粗匹配
        reg_p2p_gen = o3d.pipelines.registration.registration_generalized_icp(
            source, target, max_correspondence_distance=0.05, init=trans_init,
            estimation_method=o3d.pipelines.registration.TransformationEstimationForGeneralizedICP())  
        
        # 运行ICP算法，对应精匹配
        threshold = 0.01
        reg_p2p =  o3d.pipelines.registration.registration_icp(
            source,
            target,
            threshold,
            reg_p2p_gen.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())        
        # source_transformed = source_transformed.transform(reg_p2p.transformation)

        #累乘，将两帧间最终得到的点云的矩阵进行点乘
        T_temp_fl = reg_p2p.transformation 
        T_overall_fl = np.dot(T_temp_fl, T_overall_fl)

        csvwriter.writerow([T_temp_fl[0,3],T_temp_fl[1,3],T_temp_fl[2,3],np.sqrt(T_temp_fl[0,3]**2+T_temp_fl[1,3]**2+T_temp_fl[2,3]**2)])


    ##### ICP start

    # threshold = 0.01  # 设置一个阈值，用于剔除大于该阈值的配准误差点
    
    # # 运行ICP算法
    # reg_p2p = o3d.pipelines.registration.registration_icp(
    #     origin_pc, target, threshold, T_overall_fl,
    #     o3d.pipelines.registration.TransformationEstimationPointToPoint())
    # origin_pc
    

    # T = reg_p2p.transformation

    # print("变换矩阵 after ICP:\n", T)

    #### ICP end

    # 将变换后的点云应用于源点云
    csvfile.close()

    source_transformed = origin_pc.transform(T_overall_fl)

    # 可视化结果
    target = o3d.io.read_point_cloud(f"{folder}{150}.ply")
    target = pc_processing(target)

    print("变换矩阵 after ICP:\n", T_overall_fl)

    
    # ground_truth

    source_transformed.paint_uniform_color([1,0, 0])
    target.paint_uniform_color([0, 0, 1])
    ttt.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([source_transformed,target])