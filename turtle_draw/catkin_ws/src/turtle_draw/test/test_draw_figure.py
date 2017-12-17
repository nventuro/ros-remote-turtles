#!/usr/bin/env python

import unittest
from src import draw_figure

import math

class _TestHelperFunctions(unittest.TestCase):
    def test_angle_between_points(self):
        pass

    def test_min_angle_between_angles(self):
        pass

    def test_is_spin_clockwise(self):
        pass

    def test_are_points_equal(self):
        pass

    def test_are_angles_equal(self):
        pass

    def test_normalize_rad(self):
        # Normal usage
        self.assertAlmostEqual(draw_figure._normalize_rad(0), 0) # No offset
        self.assertAlmostEqual(draw_figure._normalize_rad(math.pi / 2), math.pi / 2) # No normalization needed (positive angle)
        self.assertAlmostEqual(draw_figure._normalize_rad(-math.pi / 2), -math.pi / 2) # No normalization needed (negative angle)

        # Edge cases
        self.assertAlmostEqual(draw_figure._normalize_rad(math.pi), math.pi) # pi is not normalized
        self.assertAlmostEqual(draw_figure._normalize_rad(-math.pi), math.pi) # -pi is normalized

        # Extreme cases
        self.assertAlmostEqual(draw_figure._normalize_rad(2 * math.pi), 0) # Large positive angle
        self.assertAlmostEqual(draw_figure._normalize_rad(-2 * math.pi), 0) # Large negative angle
        self.assertAlmostEqual(draw_figure._normalize_rad(20 * math.pi), 0) # Very large positive angle
        self.assertAlmostEqual(draw_figure._normalize_rad(-20 * math.pi), 0) # Very large negative angle

    def test_deg_to_rad(self):
        # Normal usage
        self.assertAlmostEqual(draw_figure._deg_to_rad(0), 0) # No offset
        self.assertAlmostEqual(draw_figure._deg_to_rad(180), math.pi) # Normal case
        self.assertAlmostEqual(draw_figure._deg_to_rad(-180), -math.pi) # Negative angles

        # Extreme cases
        self.assertAlmostEqual(draw_figure._deg_to_rad(180 + 360), 3 * math.pi) # Angles over 360 degrees
        self.assertAlmostEqual(draw_figure._deg_to_rad(-180 - 360), -3 * math.pi) # Negative Angles over 360 degrees

    def test_clamp(self):
        # Normal usage
        self.assertEqual(draw_figure._clamp(10, 1, 100), 10) # No clamping
        self.assertEqual(draw_figure._clamp(10, 20, 100), 20) # Clamping at bottom
        self.assertEqual(draw_figure._clamp(10, 1, 5), 5) # Clamping at top
        self.assertEqual(draw_figure._clamp(10, 5, 5), 5) # Clamping at both

        # Bad usage
        with self.assertRaises(ValueError):
            draw_figure._clamp(10, 1, 0) # Max smaller than min

if __name__ == "__main__":
    unittest.main()
