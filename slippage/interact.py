import open3d as o3d

# 加载点云数据
pcd = o3d.io.read_point_cloud("../5/117.ply")

# 创建一个可视化窗口
vis = o3d.visualization.Visualizer()
vis.create_window()

# 添加点云到窗口
vis.add_geometry(pcd)

# 创建一个空的几何体对象
# empty_geometry = o3d.geometry.Geometry()

# 获取窗口中的相机参数
param = vis.get_view_control().convert_to_pinhole_camera_parameters()

# 手动拖动点云并实时查看旋转矩阵
while True:
    vis.update_geometry(pcd)  # 将空的几何体对象传递给update_geometry方法
    vis.poll_events()
    vis.update_renderer()
    
    # 获取当前相机参数
    current_param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    
    # 比较当前参数和之前保存的参数
    if current_param != param:
        # 获取旋转矩阵并打印
        rotation_matrix = current_param.extrinsic[:3, :3]
        print("旋转矩阵：\n", rotation_matrix)
        
        # 更新参数
        param = current_param

# 关闭窗口
vis.destroy_window()
