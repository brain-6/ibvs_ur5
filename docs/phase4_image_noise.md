# 图像噪声：检测器最小接入

本文件说明图像噪声接入和运行协议，正式批量结果另行汇总。计划强度 σ=0/5/10/20，各三个固定种子（1001/1002/1003）；控制参数与基线判据不变。

## 插入位置与参数

`camera image → BGR8 → GaussianImageNoise.apply → HSV → 红/绿区域 → 中心坐标 → 原控制器`。

检测器启动参数 `gaussian_sigma`（double，默认 0.0）和 `noise_seed`（integer，默认 1001）均只读，中途改变实验条件应重启节点。每个节点启动时只初始化一次 NumPy `default_rng`，后续帧继续使用它。固定种子和相同输入/调用顺序能复现噪声序列，不保证异步 Gazebo 闭环逐帧一致。

独立地对每帧、每个 BGR 像素通道生成均值 0、标准差 σ 的高斯随机数；图像转浮点后相加，裁剪到 [0,255]，用 `np.rint` 就近取整，再转 uint8。σ 的单位是 0–255 颜色数值，不是坐标像素距离。σ 不是噪声最大幅度；裁剪、取整后的实际变化分布也不再严格等于生成的高斯分布。

σ=0 直接返回输入图像，不转换、不取整、不消耗随机数。噪声作用于完整图像，因此红目标和绿特征的检测均可能受影响；静态控制器仍锁存首次目标。结果中的控制误差不能直接当作无噪声真值误差。

## 检测器单独运行示例

只在没有另一份检测器时使用。此命令不自动启动相机、仿真或控制器：

```bash
ros2 run ibvs_ur5 detect_target.py --ros-args \
  -p gaussian_sigma:=10.0 -p noise_seed:=1001 \
  -p write_debug_image:=false
```

## 验证范围与后续

单元测试覆盖零噪声逐值不变、不消耗随机数、固定种子序列复现、连续帧变化、不同种子变化、输入保护、裁剪和取整、生成噪声量级与配置拒绝。

本步骤另用合成图像比较旧版与新版 σ=0 的实际检测回调发布结果，并检查已安装节点的噪声参数。该检查验证代码接入，不代替真实相机图像、闭环收敛或鲁棒性实验。

## 完整 trial 入口

```bash
ros2 run ibvs_ur5 run_phase4_trial.py \
  --output /tmp/noise_sigma10_seed1001_unique \
  --gaussian-sigma 10 --noise-seed 1001
```

在源码未提交的开发试跑中另加 `--allow-dirty`，此时 `formal_candidate=false`；正式采集前先提交完整功能。输出目录必须未使用过，失败也不覆盖。

`--gaussian-sigma` 会自动生成 `factor=image_noise`、`level=sigma=...`，不允许与手动填写的噪声标签矛盾；`--gaussian-sigma 0` 是该处理链的零噪声对照。不指定该选项时仍默认无噪声 baseline。强度必须有限且非负，种子必须是非负整数。

运行器生成 `detector_parameters.yaml`，启动检测器后通过 `/target_detector/get_parameters` 查询实际生效值，与请求参数逐项比较。核对不通过时不会启动运动控制器；保留错误记录。`result.json` 保存 `requested_detector_parameters`、`effective_detector_parameters`、`image_noise` 以及顶层 `seed`。旧字段 `detector_parameters` 只保留请求值供兼容，不能用它代替生效核对。

开发试跑用于确认接入链路，不作为正式噪声成功率。后续提交后按固定条件采集正式实验，数据异常和失败分别保留。误差仍基于受到噪声影响的检测坐标，不是无噪声参考真值。
