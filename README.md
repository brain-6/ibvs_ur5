IBVS 视觉伺服项目 (UR5 + ROS2 + Gazebo)



本项目旨在将硕士期间的 Matlab 视觉伺服算法，迁移并工程化到 ROS2 + Gazebo 仿真环境中。



演示视频



Phase 2: 视觉特征提取与质心发布

\[!\[Phase 2 Demo](https://img.shields.io/badge/Bilibili-Phase2\_Demo-00A1D6?logo=bilibili)](https://www.bilibili.com/video/BV1DyTJ6bEAt)



功能说明：

\- 在 Gazebo 中搭建包含 UR5 机械臂、红色目标方块和俯视相机的仿真环境。

\- 编写 ROS2 Python 节点，通过 HSV 色彩空间实时提取红色方块的像素坐标。

\- 计算图像矩（Image Moments）获取目标质心，并发布到 `/target\_centroid` 话题。



&#x20;技术栈

\- 仿真环境: Gazebo Fortress (Ignition)

\- 中间件: ROS2 Humble

\- 控制框架: ros2\_control + joint\_trajectory\_controller

\- 视觉处理: OpenCV + cv\_bridge + NumPy

\- 开发语言: Python 3



项目结构

src/ibvs\_ur5/

├── config/          # 控制器配置文件

├── launch/          # 启动文件

├── scripts/         # Python 节点 (视觉检测等)

└── worlds/          # Gazebo 自定义世界文件

