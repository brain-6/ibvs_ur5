# UR5 IBVS Visual Servoing

基于 ROS 2 Humble、Gazebo Fortress 和 `ros2_control` 的 UR5 视觉伺服仿真。固定相机检测红色目标和绿色末端特征，机械臂根据图像误差调整运动，使两个质心逐渐靠近。

项目已完成静态目标闭环控制、参数调优，以及图像噪声、相机内参偏差和控制时间配置的扰动实验。

```text
相机图像 → HSV 颜色检测 → 质心误差 → 末端速度 → 关节轨迹 → Gazebo
    ↑                                                        │
    └────────────────── 图像反馈 ────────────────────────────┘
```

控制器采用阻尼最小二乘逆运动学，并加入零空间姿态调整和关节速度限制。实验工具负责场景启动、参数核对、误差记录与结果判定。

## 演示

- [颜色检测](https://www.bilibili.com/video/BV1DyTJ6bEAt)
- [静态目标闭环控制](https://www.bilibili.com/video/BV1fhhm6LEEY/)
- [调参后的静态控制效果](https://www.bilibili.com/video/BV11zae6xEQg/)

第二段视频中段为 3 倍速，第三段保留原速。收敛时间以实验记录为准。

## 实验结果

当前配置在固定初始姿态与静态目标下，三个有效零扰动对照的平均收敛时间为 **3.620 s**。计时从首条已核实的运动命令开始；误差降至 5 px 以下并持续保持至少三秒后，取保持区间的起点作为收敛时刻。

Phase 4 测试了以下条件，每个条件计划重复三次，标称条件共用零扰动对照：

| 实验 | 设置 |
|---|---|
| 图像噪声 | 在 HSV 转换前向 BGR 图像加入高斯噪声，σ = 0 / 5 / 10 / 20 |
| 相机内参偏差 | 控制器的 fx、fy 同比例调整为标称值的 0.8 / 0.9 / 1.0 / 1.1 / 1.2 倍，实际相机保持原配置 |
| 控制频率与轨迹时长 | 20 / 10 / 5 / 2 Hz，分别配合 0.0125 / 0.025 / 0.05 / 0.125 s 的轨迹时长 |

共保存 35 次试验记录，包含建立对照时的两次重试。其中 28 次达到误差保持条件；按预定协议检查参数、观测和时钟记录后，11 次可用于正式比较，另外 24 次带有异常记录。扰动条件的有效重复仍不足，低频条件的表现有待复测。

数据、图表和逐次结果见 [Phase 4 实验报告](docs/phase4_results.md)。

早期 Phase 3 版本完成了三次静态目标测试，平均收敛时间为 81.167 s，配置保存在 `phase3-complete` 标签中。该阶段使用独立的计时口径，详细指标见 [Phase 3 报告](docs/phase3_validation.md)。

## 运行

以下以已安装 ROS 2 Humble、Gazebo Fortress、colcon 并初始化 rosdep 的 Linux 环境为例，工作空间路径为 `/root/ur_ws`。

首次获取源码时，克隆到尚不存在的目录，并一并下载 UR 机器人模型：

```bash
git clone --recurse-submodules https://github.com/brain-6/ibvs_ur5.git /root/ur_ws
cd /root/ur_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
```

绿色末端特征球由本项目的 `ur5_gazebo.xacro` 添加，生成 URDF 时会与官方 UR5 模型合并。

首次运行或更新源码后构建，并生成仿真所需的 URDF：

```bash
cd /root/ur_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ibvs_ur5 --symlink-install
source install/setup.bash

ros2 run xacro xacro \
  src/ibvs_ur5/urdf/ur5_gazebo.xacro \
  > /tmp/ur5_gazebo.urdf
```

`/tmp` 清理后需重新生成 URDF。打开三个终端，每个终端先加载环境：

```bash
cd /root/ur_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

再分别启动仿真、检测器和控制器：

```bash
# 终端 1
ros2 launch ibvs_ur5 gazebo_ur5.launch.py

# 终端 2
ros2 run ibvs_ur5 detect_target.py

# 终端 3
ros2 run ibvs_ur5 ibvs_controller.py
```

默认控制频率为 10 Hz，增益为 9.6，轨迹时长为 0.025 s。完整参数见 [静态基线配置](src/ibvs_ur5/config/phase4_static_baseline.yaml)。

### 单轮扰动实验

先关闭手动启动的仿真和节点，再运行：

```bash
ros2 run ibvs_ur5 run_phase4_trial.py \
  --gaussian-sigma 5 --noise-seed 1001 \
  --output /tmp/ibvs_noise_s5_s1001
```

运行器会启动并关闭本轮所需进程，保存参数、误差 CSV、结果 JSON 和日志。每轮使用新的输出目录；正式采集要求 Git 工作树干净。其他条件的命令见 [实验复现说明](docs/phase4_reproduction.md)，指标定义见 [记录协议](docs/phase4_protocol.md)。

### 测试

```bash
python3 -m unittest discover \
  -s src/ibvs_ur5/tests \
  -p 'test_*.py'
```

测试覆盖控制数学、日志分析、图像噪声和实验判据。

## 已知问题

- 运行环境存在时钟跳变和偶发启动超时，相关试验已保留异常标记。后续先处理运行环境，再补充有效样本。
- 当前评估集中在固定相机、固定初始姿态和静态目标场景。动态跟踪与真实机器人部署留待后续开展。

## 代码入口

- [detect_target.py](src/ibvs_ur5/scripts/detect_target.py)：颜色分割与质心提取。
- [ibvs_controller.py](src/ibvs_ur5/scripts/ibvs_controller.py)：视觉反馈控制、姿态调整与轨迹发布。
- [run_phase4_trial.py](src/ibvs_ur5/scripts/run_phase4_trial.py)：单轮实验启动、记录与清理。
