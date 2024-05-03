import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl
import numpy as np
from plyfile import PlyData, PlyElement

i = 0
folder_name = "./4/"

# def save_point_cloud_to_ply(point_cloud, file_path):
#     """
#     将三维点云数据保存为PLY文件
#     :param point_cloud: 三维点云数据，格式为Nx3的numpy数组
#     :param file_path: 保存PLY文件的路径
#     """
#     num_points = point_cloud.shape[0]

#     header = f"ply\nformat ascii 1.0\ncomment Generated by Python\nelement vertex {num_points}\nproperty float32 x\nproperty float32 y\nproperty float32 z\nend_header\n"

#     with open(file_path, 'w') as f:
#         # 写入PLY文件头
#         f.write(header)

#         # 写入点云数据
#         for i in range(num_points):
#             x, y, z = point_cloud[i]
#             f.write(f"{x} {y} {z}\n")

def point_cloud_callback(msg):
    global i
    
    # 将PointCloud2消息转换为numpy数组
    pc_data = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
    points = np.array(list(pc_data), dtype=np.float32)

    # points = points[points[:, 0]>-0.05]  # filter point clouds

    # 创建PLY文件头部
    header = """ply
format ascii 1.0
element vertex {n}
property float x
property float y
property float z
end_header
""".format(n=points.shape[0])

    # file_name = f"point_cloud_{rospy.get_rostime().to_sec()}.ply"
    file_name = f"{folder_name}{i}.ply"
    i += 1

    # 将点数据写入PLY文件
    with open(file_name, 'w') as f:
        f.write(header)
        np.savetxt(f, points, fmt='%f %f %f')

def ros_node():
    """
    ROS节点函数，初始化ROS节点并订阅点云消息
    """
    rospy.init_node('point_cloud_converter')
    rospy.Subscriber('/royale_cam0/point_cloud_0', PointCloud2, point_cloud_callback)
    rospy.spin()


if __name__ == '__main__':
    ros_node()