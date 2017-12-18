#!/usr/bin/env python

import unittest
from src import draw_figure
from src.draw_figure import Pose

import math

class TestAngleBetweenPoints(unittest.TestCase):
    # Normal usage
    def test_positive_angle(self):
       self.assertAlmostEqual(draw_figure._angle_between_points(Pose(0, 0), Pose(0, 1)), math.pi / 2)
    def test_negative_angle(self):
       self.assertAlmostEqual(draw_figure._angle_between_points(Pose(0, 0), Pose(0, -1)), -math.pi / 2)

    # Edge cases
    def test_zero_angle(self):
        self.assertAlmostEqual(draw_figure._angle_between_points(Pose(0, 0), Pose(1, 0)), 0)
    def test_max_angle(self):
        self.assertAlmostEqual(draw_figure._angle_between_points(Pose(0, 0), Pose(-1, 0)), math.pi)

class TestMinAngleBetweenAngles(unittest.TestCase):
    # Normal usage
    def test_positive_first_quadrant(self):
        self.assertAlmostEqual(draw_figure._min_angle_between_angles(math.pi * 0.125, math.pi * 0.25), math.pi * 0.125)
    def test_negative_first_quadrant(self):
        self.assertAlmostEqual(draw_figure._min_angle_between_angles(math.pi * 0.25, math.pi * 0.125), -math.pi * 0.125)
    def test_same_angle(self):
        self.assertAlmostEqual(draw_figure._min_angle_between_angles(math.pi/2, math.pi/2), 0)
    def test_same_normalized_angle_clockwise(self):
        self.assertAlmostEqual(draw_figure._min_angle_between_angles(0, -math.pi * 2), 0)
    def test_same_normalized_angle_counterclockwise(self):
        self.assertAlmostEqual(draw_figure._min_angle_between_angles(0, math.pi * 2), 0)

    # Edge cases
    def test_positive_crossing_zero(self):
        self.assertAlmostEqual(draw_figure._min_angle_between_angles(-math.pi/4, math.pi/4), math.pi/2)
    def test_negative_crossing_zero(self):
        self.assertAlmostEqual(draw_figure._min_angle_between_angles(math.pi/4, -math.pi/4), -math.pi/2)
    def test_positive_crossing_pi(self):
        self.assertAlmostEqual(draw_figure._min_angle_between_angles(math.pi * 0.875, math.pi * 1.125), math.pi * 0.25)
    def test_negative_crossing_pi(self):
        self.assertAlmostEqual(draw_figure._min_angle_between_angles(math.pi * 1.125, math.pi * 0.875), -math.pi * 0.25)
    def test_positive_crossing_2_pi(self):
        self.assertAlmostEqual(draw_figure._min_angle_between_angles(math.pi * 1.875, math.pi * 0.125), math.pi * 0.25)
    def test_negative_crossing_2_pi(self):
        self.assertAlmostEqual(draw_figure._min_angle_between_angles(math.pi * 0.125, math.pi * 1.875), -math.pi * 0.25)

class TestIsSpinClockwise(unittest.TestCase):
    # Normal usage
    def test_clockwise_spin(self):
        self.assertTrue(draw_figure._is_spin_clockwise(math.pi/2, 0))
    def test_counterclockwise_spin(self):
        self.assertFalse(draw_figure._is_spin_clockwise(0, math.pi/2))

    # Edge cases
    def test_clockwise_crossing_zero(self):
        self.assertTrue(draw_figure._is_spin_clockwise(math.pi/4, -math.pi/4))
    def test_counter_clockwise_crossing_zero(self):
        self.assertFalse(draw_figure._is_spin_clockwise(-math.pi/4, math.pi/4))
    def test_clockwise_crossing_pi(self):
        self.assertTrue(draw_figure._is_spin_clockwise(math.pi * 1.25, math.pi * 0.75))
    def test_counter_clockwise_crossing_pi(self):
        self.assertFalse(draw_figure._is_spin_clockwise(math.pi * 0.75, math.pi * 1.25))

class TestArePointsEqual(unittest.TestCase):
    tolerance = 0.01

    # Normal usage
    def test_equal_points(self):
        self.assertTrue(draw_figure._are_points_equal(Pose(0, 0), Pose(0, 0), self.tolerance))
    def test_different_points(self):
        self.assertFalse(draw_figure._are_points_equal(Pose(0, 0), Pose(1, 1), self.tolerance))

    # Edge cases
    def test_barely_equal_points(self):
        self.assertTrue(draw_figure._are_points_equal(Pose(0, 0), Pose(self.tolerance / 2, self.tolerance / 2), self.tolerance))
    def test_barely_different_points(self):
        self.assertFalse(draw_figure._are_points_equal(Pose(0, 0), Pose(self.tolerance, self.tolerance), self.tolerance))

class TestAreAnglesEqual(unittest.TestCase):
    tolerance = math.pi / 100

    # Normal usage
    def test_equal_angles(self):
        self.assertTrue(draw_figure._are_angles_equal(math.pi, math.pi, self.tolerance))
    def test_different_angles(self):
        self.assertFalse(draw_figure._are_angles_equal(0, math.pi, self.tolerance))
    def test_equal_normalized_angles(self):
        self.assertTrue(draw_figure._are_angles_equal(math.pi, -math.pi, self.tolerance))

    # Edge cases
    def test_barely_equal_angles(self):
        self.assertTrue(draw_figure._are_angles_equal(math.pi/2, math.pi/2 + 0.9 * self.tolerance, self.tolerance))
    def test_barely_different_angles(self):
        self.assertFalse(draw_figure._are_angles_equal(math.pi/2, math.pi/2 + 1.1 * self.tolerance, self.tolerance))
    def test_barely_equal_angles_when_not_normalized(self):
        self.assertTrue(draw_figure._are_angles_equal(math.pi, math.pi + 0.9 * self.tolerance, self.tolerance))

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
