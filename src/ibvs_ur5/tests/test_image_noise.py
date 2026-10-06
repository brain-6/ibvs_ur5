import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from image_noise import GaussianImageNoise


class ImageNoiseTest(unittest.TestCase):
    def test_zero_sigma_is_exact_bypass_without_rng_consumption(self):
        image = np.arange(256, dtype=np.uint8).reshape(16, 16, 1).repeat(3, axis=2)
        operator = GaussianImageNoise(0, 1001)
        before = operator.rng.bit_generator.state
        result = operator.apply(image)
        self.assertIs(result, image)
        np.testing.assert_array_equal(result, image)
        self.assertEqual(before, operator.rng.bit_generator.state)

    def test_seed_repeats_sequence_but_frames_and_other_seeds_differ(self):
        image = np.full((48, 64, 3), 128, dtype=np.uint8)
        a, b = GaussianImageNoise(10, 1001), GaussianImageNoise(10, 1001)
        first, second = a.apply(image), a.apply(image)
        np.testing.assert_array_equal(first, b.apply(image))
        np.testing.assert_array_equal(second, b.apply(image))
        self.assertFalse(np.array_equal(first, second))
        self.assertFalse(np.array_equal(first, GaussianImageNoise(10, 1002).apply(image)))
        np.testing.assert_array_equal(image, np.full_like(image, 128))

    def test_clipping_and_rounding_do_not_wrap_uint8(self):
        class FixedNoise:
            def normal(self, mean, sigma, size):
                return np.array([[[-10.0, 10.0, 0.6]]])

        operator = GaussianImageNoise(10, 1001)
        operator.rng = FixedNoise()
        result = operator.apply(np.array([[[0, 255, 100]]], dtype=np.uint8))
        np.testing.assert_array_equal(result, [[[0, 255, 101]]])
        self.assertEqual(result.dtype, np.uint8)

    def test_midrange_distribution_has_requested_scale(self):
        # 中间亮度避免裁剪干扰；检查量级、中心与通道独立性，不拟合闭环结果。
        image = np.full((256, 256, 3), 128, dtype=np.uint8)
        delta = GaussianImageNoise(10, 1001).apply(image).astype(float) - 128
        self.assertLess(abs(delta.mean()), 0.1)
        self.assertAlmostEqual(delta.std(), 10, delta=0.1)
        correlation = np.corrcoef(delta[:, :, 0].ravel(), delta[:, :, 1].ravel())[0, 1]
        self.assertLess(abs(correlation), 0.02)

    def test_invalid_configuration_and_image_are_rejected(self):
        for sigma in (-1, float('inf'), float('nan')):
            with self.subTest(sigma=sigma), self.assertRaises(ValueError):
                GaussianImageNoise(sigma, 1001)
        for seed in (-1, 1.5, True):
            with self.subTest(seed=seed), self.assertRaises(ValueError):
                GaussianImageNoise(10, seed)
        for image in (np.zeros((3, 3), dtype=np.uint8), np.zeros((3, 3, 3))):
            with self.assertRaises(ValueError):
                GaussianImageNoise().apply(image)


if __name__ == '__main__':
    unittest.main()
