# Phase 4 有限实验的复现入口

本项目是 ROS 2/Gazebo 学习与展示仿真。本轮按预定条件完成有限探索，成功、失败与环境异常均保留，不把它作为工程部署验收。

## 环境与版本

历史数据由提交 `18d2e5762ecb97df23933351d7691101bcdd6a1a` 运行，使用当时容器的 ROS 2 Humble/Gazebo Fortress。结果材料和参数配表在实验结束后整理；不能把这些后续文档提交误写成实际运行版本。每轮 `result.json` 保存真实 HEAD、脏状态、请求/生效参数、世界/URDF 哈希。

确保没有另一套本项目仿真，且 `/tmp/ur5_gazebo.urdf` 已按 README 生成。在容器中：

```bash
cd /root/ur_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
git status -sb
```

每轮输出目录必须未使用过。运行器默认拒绝脏工作树；`--allow-dirty` 只用于开发试跑，不能标成正式干净样本。单轮命令会启动并关闭自己的仿真、检测器、控制器，不需手动另启它们。

## 1. 图像噪声

σ 依次为 0、5、10、20，每档 seed=1001/1002/1003。单轮例子：

```bash
ros2 run ibvs_ur5 run_phase4_trial.py \
  --output /tmp/my_noise_sigma5_seed1001 \
  --gaussian-sigma 5 --noise-seed 1001
```

运行器会自动生成噪声因素标签，查询检测器生效参数；不是只给结果贴标签。σ=0 不生成噪声，三次重复用来观察运行变化。噪声加在 BGR→HSV 之前，红目标与绿特征都可能受影响。

## 2. 控制器内参偏差

只改变控制器使用的 fx/fy，实际相机和估计深度不变。所有配置保存完整参数，便于人工核对；相对基线只修改 fx/fy 两项。

| 倍率 | fx=fy | 配置文件 |
|---:|---:|---|
| 0.8 | 443.504 | intrinsics_scale080.yaml |
| 0.9 | 498.942 | intrinsics_scale090.yaml |
| 1.0 | 554.380 | 原 phase4_static_baseline.yaml，可复用同条件对照 |
| 1.1 | 609.818 | intrinsics_scale110.yaml |
| 1.2 | 665.256 | intrinsics_scale120.yaml |

单轮例子，在 `/root/ur_ws` 中执行：

```bash
ros2 run ibvs_ur5 run_phase4_trial.py \
  --params src/ibvs_ur5/config/phase4_sweeps/intrinsics_scale080.yaml \
  --factor intrinsics --level scale=0.8 \
  --noise-seed 1001 --output /tmp/my_intrinsics080_repeat01
```

每个非基线档位各运行三次，分别使用不同输出目录及 1001/1002/1003；此处 sigma 默认 0，种子不产生图像扰动。factor/level 是描述，实际行为由 YAML 决定，必须核对结果中的有效 fx/fy。

## 3. 控制周期与匹配轨迹时长

| 控制频率 | 轨迹时长 / s | 配置文件 |
|---:|---:|---|
| 20 Hz | 0.0125 | frequency_hz20.yaml |
| 10 Hz | 0.025 | 原 phase4_static_baseline.yaml，可复用同条件对照 |
| 5 Hz | 0.05 | frequency_hz5.yaml |
| 2 Hz | 0.125 | frequency_hz2.yaml |

轨迹时长/控制周期保持 0.25。除这两个预定绑定参数外，其他控制参数不变。该实验是联合配置影响，不能叫纯采样频率效应。

```bash
ros2 run ibvs_ur5 run_phase4_trial.py \
  --params src/ibvs_ur5/config/phase4_sweeps/frequency_hz20.yaml \
  --factor frequency_duration --level 'rate=20Hz,duration=0.0125s' \
  --noise-seed 1001 --output /tmp/my_frequency20_repeat01
```

每个非基线档位各三次。已有三次配置、代码、处理链和判据一致的零扰动对照可以共享；不能仅因都叫“baseline”就复用。

## 结果判读与失败保留

- 必须在首条已核实运动命令之后 30 秒内，完成误差严格小于 5 px 的至少三秒新鲜观测保持；收敛时间取合格保持区间起点。
- `geometric_converged` 只说明观测误差保持达标。
- `formal_candidate` 表示符合既定正式比较条件；正常观测超时的有效失败也应为 true，并计入失败分母。
- 时钟、stale、服务超时等异常单列保留。非零退出码不一定是脚本崩溃，先读 `end_reason` 和 `anomalies`。
- 不覆盖失败目录，不为得到成功而修改参数。发现残留进程或退出清理失败时停止后续启动，避免两个场景重叠。

判据和字段细节见 [记录协议](phase4_protocol.md)、[图像噪声说明](phase4_image_noise.md)。
