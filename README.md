# UR5 IBVS Visual Servoing

基于 ROS 2 Humble、Gazebo Fortress 和 `ros2_control` 的 UR5 图像视觉伺服项目。摄像机检测红色目标与绿色末端特征，控制器在图像平面减小二者的质心误差，并通过速度级逆运动学生成关节轨迹命令。

## 项目状态

- Phase 2：红色目标与绿色特征检测，发布图像质心。
- Phase 3：静态目标闭环 IBVS，已完成 Gazebo 基线验收。
- Phase 4：噪声、相机参数偏差和控制频率鲁棒性实验，计划中。

Phase 2 演示：[Bilibili](https://www.bilibili.com/video/BV1DyTJ6bEAt)

## Phase 3 基线结果

在相同初始姿态下完成 3 次默认参数实验：

| 指标 | 结果 |
|---|---:|
| 成功率 | 3 / 3 |
| 平均收敛时间 | 81.167 s |
| 收敛时间样本标准差 | 1.005 s |
| 最终死区误差 | 4.2 px |
| 图像路径效率 | 0.9951 |
| 最大横向偏差 | 3.575 px |
| 输入超时次数 | 0 |

这里的路径效率只描述静态目标下的二维图像误差轨迹，不代表关节空间或三维末端轨迹是全局最短路径。完整方法、参数、复现实验和限制见 [Phase 3 验收报告](docs/phase3_validation.md)。

## 默认控制参数

```text
lambda_gain=0.8
estimated_depth=2.0
max_cartesian_speed=0.06
max_joint_speed=0.60
damping=0.02
posture_gain=0.25
deadzone_px=5.0
control_rate=10.0
trajectory_duration=0.11
static_target=true
```

## 快速运行

容器重新创建或 `/tmp` 被清理后，先生成 URDF：

```bash
cd /root/ur_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run xacro xacro \
  src/ibvs_ur5/urdf/ur5_gazebo.xacro \
  > /tmp/ur5_gazebo.urdf
```

依次在不同终端运行：

```bash
ros2 launch ibvs_ur5 gazebo_ur5.launch.py
```

```bash
ros2 run ibvs_ur5 detect_target.py
```

```bash
ros2 run ibvs_ur5 ibvs_controller.py
```

分析控制器日志：

```bash
python3 src/ibvs_ur5/scripts/analyze_ibvs_log.py /path/to/controller.log
```

运行测试：

```bash
python3 -m unittest discover \
  -s src/ibvs_ur5/tests \
  -p 'test_*.py'
```

## 目录结构

```text
src/ibvs_ur5/
|-- config/       ros2_control 配置
|-- launch/       Gazebo 与机器人启动文件
|-- scripts/      检测、控制、数学和日志分析脚本
|-- tests/        纯数学与日志分析单元测试
|-- urdf/         UR5 与仿真控制描述
`-- worlds/       Gazebo 场景
```

## 适用边界

当前结论仅针对 Gazebo 中的静态红色目标。动态目标在持续可见时可以更新；完全遮挡后只能超时保持，尚未实现预测。项目也尚未完成真实机器人所需的关节位置、加速度、碰撞、急停和硬件安全验证。

