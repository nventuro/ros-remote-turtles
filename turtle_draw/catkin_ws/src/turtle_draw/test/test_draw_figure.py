#!/usr/bin/env python

import unittest
from src import draw_figure

import math

class TestNormalizeRad(unittest.TestCase):
    # Normal usage
    def test_no_offset(self):
        self.assertAlmostEqual(draw_figure._normalize_rad(0), 0)
    def test_no_normalization_positive_angle(self):
        self.assertAlmostEqual(draw_figure._normalize_rad(math.pi / 2), math.pi / 2)
    def test_no_normalization_negative_angle(self):
        self.assertAlmostEqual(draw_figure._normalize_rad(-math.pi / 2), -math.pi / 2)

    # Edge cases
    def test_no_normalization_pi(self):
        self.assertAlmostEqual(draw_figure._normalize_rad(math.pi), math.pi)
    def test_normalization_minus_pi(self):
        self.assertAlmostEqual(draw_figure._normalize_rad(-math.pi), math.pi)

    # Extreme cases
    def test_large_positive_angle(self):
        self.assertAlmostEqual(draw_figure._normalize_rad(1.5 * math.pi), -0.5 * math.pi)
    def test_large_negative_angle(self):
        self.assertAlmostEqual(draw_figure._normalize_rad(-1.5 * math.pi), 0.5 * math.pi)
    def test_very_large_positive_angle(self):
        self.assertAlmostEqual(draw_figure._normalize_rad(20 * math.pi), 0)
    def test_very_large_negative_angle(self):
        self.assertAlmostEqual(draw_figure._normalize_rad(-20 * math.pi), 0)

class TestDegToRad(unittest.TestCase):
    # Normal usage
    def test_no_offset(self):
        self.assertAlmostEqual(draw_figure._deg_to_rad(0), 0)
    def test_positive_angle(self):
        self.assertAlmostEqual(draw_figure._deg_to_rad(180), math.pi)
    def test_negative_angle(self):
        self.assertAlmostEqual(draw_figure._deg_to_rad(-180), -math.pi)

    # Extreme cases
    def test_large_positive_angle(self):
        self.assertAlmostEqual(draw_figure._deg_to_rad(180 + 360), 3 * math.pi)
    def test_large_negative_angle(self):
        self.assertAlmostEqual(draw_figure._deg_to_rad(-180 - 360), -3 * math.pi)

class TestClamp(unittest.TestCase):
    # Normal usage
    def test_no_clamp(self):
        self.assertEqual(draw_figure._clamp(10, 1, 100), 10)
    def test_clamp_min(self):
        self.assertEqual(draw_figure._clamp(10, 20, 100), 20)
    def test_clamp_max(self):
        self.assertEqual(draw_figure._clamp(10, 1, 5), 5)
    def test_clamp_both(self):
        self.assertEqual(draw_figure._clamp(10, 5, 5), 5)

    # Bad usage
    def test_clamp_max_smaller_than_min(self):
        with self.assertRaises(ValueError):
            draw_figure._clamp(10, 1, 0)
