#!/usr/bin/env python3
"""BGR8 图像的高斯噪声；不依赖 ROS，便于单独验证。"""
import math
from numbers import Integral

import numpy as np


class GaussianImageNoise:
    def __init__(self, sigma=0.0, seed=1001):
        self.sigma = float(sigma)
        if not math.isfinite(self.sigma) or self.sigma < 0:
            raise ValueError('gaussian_sigma must be finite and nonnegative')
        if isinstance(seed, bool) or not isinstance(seed, Integral) or seed < 0:
            raise ValueError('noise_seed must be a nonnegative integer')
        self.seed = int(seed)
        # 每个节点生命周期只初始化一次；后续帧继续使用同一个生成器。
        self.rng = np.random.default_rng(self.seed)

    def apply(self, bgr):
        if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError('expected a uint8 image with shape (height, width, 3)')
        if self.sigma == 0.0:
            # 零噪声旁路：不变更像素，也不消耗随机数。
            return bgr
        noise = self.rng.normal(0.0, self.sigma, size=bgr.shape)
        # 先转浮点再相加，避免 uint8 的溢出回绕。每个像素、每个通道独立采样。
        noisy = bgr.astype(np.float64) + noise
        return np.rint(np.clip(noisy, 0.0, 255.0)).astype(np.uint8)
