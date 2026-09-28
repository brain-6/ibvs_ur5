# UR5 IBVS Visual Servoing

基于 ROS 2 Humble、Gazebo Fortress 和 `ros2_control` 的 UR5 图像视觉伺服项目。摄像机检测红色目标与绿色末端特征，控制器在图像平面减小二者的质心误差，并通过速度级逆运动学生成关节轨迹命令。

## 项目状态

- Phase 2：红色目标与绿色特征检测，发布图像质心。
- Phase 3：静态目标闭环 IBVS，已完成 Gazebo 基线验收。
- Phase 4：已完成一轮静态目标提速调参及原速演示；噪声、相机参数偏差和控制频率鲁棒性实验仍在计划中。

当前调速分支使用新的静态仿真默认参数，`phase3-complete` 标签保持原配置。本分支尚不代表 Phase 4 鲁棒性验收完成。

新参数在既定静态目标和初始姿态下三次复验，稳定进入 5 px 死区用时 3.04–3.82 秒，最终误差 2.83–3.61 px，均无特征超时警告；连续保持要求分别为 3、5、5 秒。时间使用外部单调时钟，从首次观测到轨迹命令起算，不含节点启动等待和额外保持时间，与旧日志分析器的计时口径不同。修改默认值后重新构建及 10 项单元测试通过，并完成一次无参数覆盖演示验证。

以上仅是有限参数搜索后的静态仿真结果，不证明动态跟踪、扰动鲁棒性或真实机器人安全。运行环境仍曾出现系统时间跳变，控制器自身计时逻辑未修改；正式 Phase 4 实验前需重新建立基线。

## 演示视频

- [Phase 2：红绿目标检测](https://www.bilibili.com/video/BV1DyTJ6bEAt)
- [Phase 3：静态目标闭环控制演示](https://www.bilibili.com/video/BV1fhhm6LEEY/)
- [Phase 4：静态目标提速演示（原速）](https://www.bilibili.com/video/BV11zae6xEQg/)

Phase 3 视频中段为 3 倍速；Phase 4 提速视频仅裁去首尾，闭环运动过程连续保留、未加速。视频播放时长不能直接用于比较两阶段的收敛时间，实验计时口径见上文。

Phase 4 视频中 Gazebo 主视角相对流畅，rqt 固定相机俯视图像窗口在实时运行时存在间歇性卡顿，具体原因尚未定位。本视频展示静态目标提速效果，不代表 Phase 4 鲁棒性实验或动态目标跟踪验收完成。

## Phase 3 基线结果

在相同初始姿态下完成 3 次 Phase 3 冻结默认参数实验（并非本调速分支的新默认值）：

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
lambda_gain=9.6
estimated_depth=2.0
max_cartesian_speed=0.72
max_joint_speed=3.0
damping=0.02
posture_gain=0.25
deadzone_px=5.0
control_rate=10.0
trajectory_duration=0.025
static_target=true
```

上述参数仅针对已测试的 Gazebo 静态目标。需要运行原 Phase 3 参数对照时，对控制器增加以下覆盖项：

```bash
ros2 run ibvs_ur5 ibvs_controller.py --ros-args \
  -p lambda_gain:=0.8 -p max_cartesian_speed:=0.06 \
  -p max_joint_speed:=0.6 -p trajectory_duration:=0.11
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

Phase 4 的最小记录器、固定成功判据与单轮复现命令见
[实验记录协议](docs/phase4_protocol.md)。工具具备记录能力不代表三类鲁棒性实验已经完成。

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
