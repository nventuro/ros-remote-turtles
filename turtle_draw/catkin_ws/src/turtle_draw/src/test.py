#!/usr/bin/env python

import math

def deg_to_rad(deg):
    return math.pi * deg / 180

def clamp(val, min_val, max_val):
    if min_val > max_val: raise ValueError
    return max(min_val, min(val, max_val))

import unittest

class TestHelperFunctions(unittest.TestCase):
    def test_clamp(self):
        self.assertEqual(clamp(10, 1, 100), 10)
        self.assertEqual(clamp(10, 20, 100), 20)
        self.assertEqual(clamp(10, 1, 5), 5)
        with self.assertRaises(ValueError):
            clamp(10, 1, 0)

    def test_deg_to_rad(self):
        self.assertEqual(deg_to_rad(0), 0)

if __name__ == "__main__":
    unittest.main()
