import open3d as o3d
import numpy as np
import time

folder = "../5/"

pc_frame_num = 55

def pc_processing(pc):
    points = np.asarray(pc.points)
    points = points[points[:, 0] > -0.05]
    points = points[points[:, 0] < 0.3]
    points = points[points[:, 2] < 0.73]
    pc.points = o3d.utility.Vector3dVector(points)
    return pc

# 创建一个空的点云列表
point_clouds = []

# 读取所有点云并加入列表
for i in range(pc_frame_num):
    pc = o3d.io.read_point_cloud(f"{folder}{i}.ply")
    pc = pc_processing(pc)
    point_clouds.append(pc)

# 创建动画对象
animation = o3d.visualization.draw_geometries_with_animation_callback(
    point_clouds, window_name="Open3D Animation", width=800, height=600
)

# 设置动画帧率
animation.play(duration=5000, fps=10)  # 播放持续时间 5000 毫秒，帧率 10

# 等待动画完成
while not animation.is_done():
    time.sleep(0.5)
