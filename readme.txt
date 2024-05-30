slippage：滑移数据+代码
sinkage.bag：沉陷数据
cloud2ply.py：接收点云数据并保存为ply文件
pmdCameraRos.zip：运行相机的代码


滑移部分的检测点
1. 先结合车轮深陷率，然后查丁亮老师（车辙）论文公式，看沉陷率对滑移率的影响
2. 可以考虑apriltag放在车轮上，相机识别QRcode，可以直接得出来旋转矩阵R