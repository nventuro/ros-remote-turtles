#!/usr/bin/env python

import unittest
from src import draw_figure
from src.draw_figure import Pose

import math

class Speed():
    def __init__(self, linear, angular):
        self.linear = linear
        self.angular = angular

    def __str__(self):
        return "Speed (linear: %.2f angular: %.2f)" % (self.linear, self.angular)

    def __repr__(self):
        return str(self)

class MockDrawer():
    def __init__(self, start_x, start_y, start_theta, slowdown):
        self.pose = Pose(start_x, start_y, start_theta)
        self.speed = Speed(0, 0)
        self.drawing = False
        self.slowdown = float(slowdown)

        self.drawed_points = []

    def step(self):
        if self.drawing:
            self.drawed_points.append(self.pose)

        # First spin, then move forward
        # All speeds are divided by a slowdown factor (representing a high refresh rate)
        self.pose.theta += self.speed.angular / self.slowdown
        self.pose.x += math.cos(self.pose.theta) * self.speed.linear / self.slowdown
        self.pose.y += math.sin(self.pose.theta) * self.speed.linear / self.slowdown

    def draw(self, on):
        self.drawing = on

    def __str__(self):
        return "Drawer (%s, %s, drawing: %s)" % (self.pose, self.speed, self.drawing)

    def __repr__(self):
        return str(self)

class TestMockDrawer(unittest.TestCase):
    slowdown = 100

    def setUp(self):
        self.d = MockDrawer(0, 0, 0, self.slowdown)

    # Simple cases
    def test_no_movement(self):
        self._advance_drawer()
        self._assert_drawer_pose_equal_to(Pose(0, 0, 0))

    def test_move_forward(self):
        self.d.speed.linear = 1

        self._advance_drawer()
        self._assert_drawer_pose_equal_to(Pose(1, 0, 0))

    def test_spin(self):
        self.d.speed.angular = 1

        self._advance_drawer()
        self._assert_drawer_pose_equal_to(Pose(0, 0, 1))

    # Complex cases
    def test_rotate(self):
        # Start at (1, 0) with an angle of pi/2
        self.d.pose.x = 1
        self.d.pose.theta = math.pi/2

        # These speeds will cause the object to describe a movement of a quarter of a circle
        self.d.speed.linear = math.pi/2
        self.d.speed.angular = math.pi/2

        self._advance_drawer()
        self._assert_drawer_pose_equal_to(Pose(0, 1, math.pi), delta=0.01)

    def test_advance_and_return(self):
        self.d.speed.linear = 1
        self._advance_drawer()

        self.d.speed.linear = -1
        self._advance_drawer()

        self._assert_drawer_pose_equal_to(Pose(0, 0 ,0))

    # Drawing
    def test_move_forward_no_draw(self):
        self.d.speed.linear = 1

        self._advance_drawer()
        self.assertEqual(len(self.d.drawed_points), 0)

    def test_move_forward_draw(self):
        self.d.speed.linear = 1
        self.d.draw(True)

        self._advance_drawer()

        self.assertEqual(len(self.d.drawed_points), self.slowdown)
        for p in self.d.drawed_points:
            self.assertTrue(0.0 <= p.x <= 1.01)
            self.assertEqual(p.y, 0)
            self.assertEqual(p.theta, 0)

    # Helpers
    def _advance_drawer(self):
        for n in range(self.slowdown):
            self.d.step()

    def _assert_drawer_pose_equal_to(self, p, delta=None):
        self.assertAlmostEqual(self.d.pose.x, p.x, delta=delta)
        self.assertAlmostEqual(self.d.pose.y, p.y, delta=delta)
        self.assertAlmostEqual(self.d.pose.theta, p.theta, delta=delta)
