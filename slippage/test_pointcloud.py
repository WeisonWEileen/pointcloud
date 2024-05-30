import numpy as np
import open3d as o3d
import time
import rotation_matrix


# 创建一个Open3D的点云对象
pcd = o3d.geometry.PointCloud()
folder = "../5/"
i = 0


def cal_T(axis, angle):
    if axis == 0:
        T = np.array([[1, 0, 0],
                      [0, np.cos(angle), -np.sin(angle)],
                      [0, np.sin(angle), np.cos(angle)]])
    elif axis == 1:
        T = np.array([[np.cos(angle), 0, np.sin(angle)],
                      [0, 1, 0],
                      [-np.sin(angle), 0, np.cos(angle)]])
    elif axis == 2:
        T = np.array([[np.cos(angle), -np.sin(angle), 0],
                      [np.sin(angle), np.cos(angle), 0],
                      [0, 0, 1]])
        
    return T

def update_point_cloud(vis):
    global i
    # if i == 80: return
    # if i == 80: vis.destroy_window()
    if i == 421: i = 0

    pcd = o3d.io.read_point_cloud(f"{folder}{i}.ply")


    pcd = rotation_matrix.pc_processing(pcd)
    

    points = np.asarray(pcd.points)

    # T1 = cal_T(0, np.pi/2)
    # T2 = cal_T(1, -np.pi/4)
    # T3 = cal_T(0, np.pi/4)
    # T2 = np.eye(3)
    # T3 = np.eye(3)

    # T = np.dot(T3, np.dot(T2, T1))

    # points = np.dot(T, points.T).T
    pcd.points = o3d.utility.Vector3dVector(points)


    # pc to mesh and smooth
    # pcd = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 0.01)
    # pcd = pcd.filter_smooth_laplacian(number_of_iterations=50)


    # 更新可视化界面
    vis.clear_geometries()
    vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

    i += 1

    # time.sleep(0.2)



if __name__ == "__main__":
    # 创建Visualizer对象
    vis = o3d.visualization.Visualizer()

    # 打开可视化界面
    vis.create_window()

    # 设置可视化界面的回调函数
    vis.register_animation_callback(update_point_cloud)

    # 添加点云对象到可视化界面
    vis.add_geometry(pcd)

    # 开始可视化循环
    vis.run()

    # 关闭可视化界面
    vis.destroy_window()

