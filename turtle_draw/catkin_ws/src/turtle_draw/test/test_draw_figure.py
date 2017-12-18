#!/usr/bin/env python

import unittest
from src import draw_figure
from src.draw_figure import Pose

from functools import partial
from copy import copy
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

        self.drawn_points = []

    def step(self):
        if self.drawing:
            self.drawn_points.append(copy(self.pose)) # Story a cope of the pose, not a reference

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

def point_between_points(p, p1, p2, cross_delta, dist_delta):
    # To determine if p is in the line described by p1 and p2, we can simply
    # check if the cross product between p1->p and p1->p2 is zero, that is, if
    # the angle between the vectors is zero (assuming p1 != p2)

    vec_a_x = p.x - p1.x
    vec_a_y = p.y - p1.y

    vec_b_x = p2.x - p1.x
    vec_b_y = p2.y - p1.y

    if abs(vec_a_x * vec_b_y - vec_a_y * vec_b_x) > cross_delta:
        return False

    # We now know p is in the line, but we still need to check if it is
    # between p1 and p2, by checking the coordinates
    if (abs(vec_a_x) >= abs(vec_a_y)): # If the slope of the line is mostly horizontal
        if vec_a_x > 0: # Check if the point's x-coordinates lie between p1 and p2, or very close to them
            return (p1.x <= p.x <= p2.x) or any([abs(p.x - other_p.x) < dist_delta for other_p in [p1, p2]])
        else:
            return (p2.x <= p.x <= p1.x) or any([abs(p.x - other_p.x) < dist_delta for other_p in [p1, p2]])
    else:
        if vec_a_y > 0: # Check if the point's y-coordinates lie between p1 and p2, or very close to them
            return (p1.y <= p.y <= p2.y) or any([abs(p.y - other_p.y) < dist_delta for other_p in [p1, p2]])
        else:
            return (p2.y <= p.y <= p1.y) or any([abs(p.y - other_p.y) < dist_delta for other_p in [p1, p2]])

def get_points_in_line(p1, p2, total_points):
    points = []

    theta = draw_figure._angle_between_points(p1, p2)
    distance = math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    for i in range(total_points):
        x = (p1.x + math.cos(theta) * i * distance / (total_points - 1))
        y = (p1.y + math.sin(theta) * i * distance / (total_points - 1))

        points.append(Pose(x, y))

    return points

def get_figure_key_points(f, inter_vertexes_points):
    key_points = []

    for n in range(len(f)):
        (p1, p2) = (f[n], f[(n + 1) % len(f)])
        key_points += get_points_in_line(p1, p2, inter_vertexes_points)

    return key_points

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
        self.assertEqual(len(self.d.drawn_points), 0)

    def test_move_forward_draw(self):
        self.d.speed.linear = 1
        self.d.draw(True)

        self._advance_drawer()

        self.assertEqual(len(self.d.drawn_points), self.slowdown)
        for p in self.d.drawn_points:
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

class TestPointBetweenPoints(unittest.TestCase):
    cross_delta = 0.01
    dist_delta = 0.01

    # Simple cases
    def test_point_at_start(self):
        self.assertTrue(point_between_points(Pose(0, 0), Pose(0, 0), Pose(0, 1), self.cross_delta, self.dist_delta))
    def test_point_at_end(self):
        self.assertTrue(point_between_points(Pose(0, 1), Pose(0, 0), Pose(0, 1), self.cross_delta, self.dist_delta))
    def test_point_in_middle_x(self):
        self.assertTrue(point_between_points(Pose(0, 0.5), Pose(0, 0), Pose(0, 1), self.cross_delta, self.dist_delta))
    def test_point_in_middle_x_reverse(self):
        self.assertTrue(point_between_points(Pose(0, 0.5), Pose(0, 1), Pose(0, 0), self.cross_delta, self.dist_delta))
    def test_point_in_middle_y(self):
        self.assertTrue(point_between_points(Pose(0.5, 0), Pose(0, 0), Pose(1, 0), self.cross_delta, self.dist_delta))
    def test_point_in_middle_y_reverse(self):
        self.assertTrue(point_between_points(Pose(0.5, 0), Pose(1, 0), Pose(0, 0), self.cross_delta, self.dist_delta))
    def test_point_outside(self):
        self.assertFalse(point_between_points(Pose(2, 0), Pose(0, 0), Pose(0, 1), self.cross_delta, self.dist_delta))

    # Complex cases
    def test_point_in_line_not_between_smaller_x(self):
        self.assertFalse(point_between_points(Pose(0, -1), Pose(0, 0), Pose(0, 1), self.cross_delta, self.dist_delta))
    def test_point_in_line_not_between_larger_x(self):
        self.assertFalse(point_between_points(Pose(0, 2), Pose(0, 0), Pose(0, 1), self.cross_delta, self.dist_delta))
    def test_point_in_line_not_between_smaller_y(self):
        self.assertFalse(point_between_points(Pose(-1, 0), Pose(0, 0), Pose(1, 0), self.cross_delta, self.dist_delta))
    def test_point_in_line_not_between_larger_y(self):
        self.assertFalse(point_between_points(Pose(2, 0), Pose(0, 0), Pose(1, 0), self.cross_delta, self.dist_delta))

class TestGetPointsInLine(unittest.TestCase):
    total_points = 10
    cross_delta = 0.01
    dist_delta = 0.01

    # Simple cases
    def test_amount_of_points(self):
        self.assertEqual(len(get_points_in_line(Pose(0, 0), Pose(1, 0), self.total_points)), self.total_points)
    def test_points_in_horizontal_line(self):
        self._test_points_in_line(Pose(0, 0), Pose(1, 0))
    def test_points_in_vertical_line(self):
        self._test_points_in_line(Pose(0, 0) ,Pose(0, 1))
    def test_points_in_diagonal_line(self):
        self._test_points_in_line(Pose(0, 0), Pose(1, 1))

    # Helpers
    def _test_points_in_line(self, p1, p2):
        for p in get_points_in_line(p1, p2, self.total_points):
            self.assertTrue(point_between_points(p, p1, p2, self.cross_delta, self.dist_delta))

class TestGetFigureKeyPoints(unittest.TestCase):
    inter_vertexes_points = 10
    cross_delta = 0.01
    dist_delta = 0.01

    # Simple cases
    def test_amount_of_points(self):
        f = [Pose(0, 0), Pose(1, 0), Pose(0.5, 1)]
        self.assertEqual(len(get_figure_key_points(f, self.inter_vertexes_points)), len(f) * self.inter_vertexes_points)

    def test_line_key_points(self):
        self._test_points_in_at_least_one_outline_line([Pose(0, 0), Pose(1, 0)])

    def test_triangle_key_points(self):
        self._test_points_in_at_least_one_outline_line([Pose(0, 0), Pose(1, 0), Pose(0.5, 1)])

    def test_square_key_points(self):
        self._test_points_in_at_least_one_outline_line([Pose(0, 0), Pose(1, 0), Pose(1, 1), Pose(0, 1)])

    # Helpers
    def _test_points_in_at_least_one_outline_line(self, f):
        for p in get_figure_key_points(f, self.inter_vertexes_points):
            self.assertTrue(any([point_between_points(p, f[n], f[(n + 1) % len(f)], self.cross_delta, self.dist_delta) for n in range(len(f))]))

class TestDrawFigure(unittest.TestCase):
    inter_vertexes_points = 10
    cross_delta = 0.1
    dist_delta = 0.15

    def setUp(self):
        self.d = MockDrawer(0, 0, 0, 10)

        self.deps = {
            "log"       : self._log,
            "pause"     : self._pause,
            "abort"     : self._abort,
            "step"      : partial(self._step, drawer=self.d),
            "curr_pose" : partial(self._curr_pose, drawer=self.d),
            "move"      : partial(self._move, drawer=self.d),
            "pen"       : partial(self._pen, drawer=self.d)
        }

    def test_draw_incomplete_triangle(self):
        f = [Pose(0, 0), Pose(1, 0), Pose(0.5, 1)]
        # draw only goes from point to point, and doesn't go from the last point
        # to the first one: if this is not done manually, the drawn figure will
        # not match
        draw_figure.draw(f, self.deps)

        self.assertFalse(self._points_match_figure(f, self.d.drawn_points))

    def test_draw_triangle(self):
        f = [Pose(0, 0), Pose(1, 0), Pose(0.5, 1)]
        draw_figure.draw(self._get_complete_figure(f), self.deps)

        self.assertTrue(self._points_match_figure(f, self.d.drawn_points))

    def test_draw_square(self):
        f = [Pose(0, 0), Pose(1, 0), Pose(1, 1), Pose(0, 1)]
        draw_figure.draw(self._get_complete_figure(f), self.deps)

        self.assertTrue(self._points_match_figure(f, self.d.drawn_points))

    def test_draw_triangle_not_square(self):
        f = [Pose(0, 0), Pose(1, 0), Pose(0.5, 1)]
        draw_figure.draw(self._get_complete_figure(f), self.deps)

        self.assertFalse(self._points_match_figure([Pose(0, 0), Pose(1, 0), Pose(1, 1), Pose(0, 1)], self.d.drawn_points))

    # Helpers
    def _get_complete_figure(self, f):
        # We add the first point to the figure, to create the segment f[-1] -> f[0]
        return f + [f[0]]

    def _points_match_figure(self, f, drawn_points):
        # For the points to correctly represent the figure, two conditions must be true:

        # a) All key points of the figure must have at least one corresponding point close to them
        for kp in get_figure_key_points(f, self.inter_vertexes_points):
            if not any([draw_figure._are_points_equal(p, kp, self.dist_delta) for p in drawn_points]):
                return False

        # b) All points must lie between two vertexes of the figure
        for p in drawn_points:
            if not any([point_between_points(p, f[n], f[(n + 1) % len(f)], self.cross_delta, self.dist_delta) for n in range(len(f))]):
                return False

        return True

    # Mocked dependencies
    def _log(self, *args):
        pass

    def _pause(self):
        return False

    def _abort(self):
        return False

    def _step(self, drawer):
        drawer.step()

    def _curr_pose(self, drawer):
        return drawer.pose

    def _move(self, linear_speed, angular_speed, drawer):
        drawer.speed.linear = linear_speed
        drawer.speed.angular = angular_speed

    def _pen(self, on, drawer):
        drawer.draw(on)
