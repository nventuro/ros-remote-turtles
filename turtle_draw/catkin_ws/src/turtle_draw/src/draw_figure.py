#!/usr/bin/env python

import math
from functools import partial
import time

import unittest

# Each turtl's position - this is set by the subsriber callbacks
turtle_poses = {}

def draw_figure(points, turtle_name):
    rate = rospy.Rate(10) # 10 Hz

    pub = rospy.Publisher("/%s/cmd_vel" % turtle_name, Twist, queue_size=10)

    while turtle_name not in turtle_poses: # We need to sleep for a bit to let the subscriber fetch the current pose at least once
        time.sleep(0)

    # We need to turn of the pen while we reach the first point
    pen_off(set_pen)
    move_straight(Pose(x=points[0].x, y=points[0].y), 1, 1, 0.1, rate, pub, turtle_name)

    # And then turn it back on to draw the different segments
    pen_on(set_pen)
    for point in points[1:]:
        move_straight(Pose(x=point.x, y=point.y), 1, 1, 0.1, rate, pub, turtle_name)

    rospy.loginfo("Turtle \"%s\" is done!" % turtle_name)

def move_straight(target, move_speed, spin_speed, pos_tolerance, rate, pub, turtle_name):
    rospy.loginfo("[%s] Going to x: %.2f y: %.2f" % (turtle_name, target.x, target.y))

    # We first spin, then move forward

    target_theta = _angle_between_points(turtle_poses[turtle_name], target)

    rospy.loginfo("[%s] Need to aquire theta: %.2f" % (turtle_name, target_theta))
    while not _are_angles_equal(turtle_poses[turtle_name].theta, target_theta, _deg_to_rad(5)) and not rospy.is_shutdown():
        pause_if_required(pub)

        spin(spin_speed, _is_spin_clockwise(turtle_poses[turtle_name].theta, target_theta), pub)
        rate.sleep()

    rospy.loginfo("[%s] Done spinning!" % turtle_name)
    stop(pub) # Stop spinning

    rospy.loginfo("[%s] Moving to the target" % turtle_name)
    # We don't move in a straight line because we might need to do small angle corrections
    move(target, move_speed, spin_speed, pos_tolerance, rate, pub, turtle_name)

    rospy.loginfo("[%s] Reached the target!" % turtle_name)
    stop(pub) # Stop moving

def move(target, move_speed, spin_speed, pos_tolerance, rate, pub, turtle_name):
    while not _are_points_equal(turtle_poses[turtle_name], target, pos_tolerance) and not rospy.is_shutdown():
        pause_if_required(pub)

        # We move forward, but may need to apply small angle corrections while doing so
        target_theta = _angle_between_points(turtle_poses[turtle_name], target)
        angle_correction = _min_angle_between_angles(turtle_poses[turtle_name].theta, target_theta)

        # We could apply some proportional gain to angle_correction here, but too much
        # will make us zigzag
        target_spin_speed = _clamp(angle_correction, -spin_speed, spin_speed)

        pub.publish(Twist(linear=Point(move_speed, 0, 0), angular=Point(0, 0, target_spin_speed)))
        rate.sleep()

def spin(speed, clockwise, pub):
    pub.publish(Twist(linear=Point(0, 0, 0), angular=Point(0, 0, speed * (-1 if clockwise else 1))))

def stop(pub):
    pub.publish(Twist(linear=Point(0, 0, 0), angular=Point(0, 0, 0)))

def _angle_between_points(point_a, point_b):
    # math.atan2 works correctly on all 4 quadrants, and we don't need to
    # worry about dividing by zero when the x coordinates match
    return math.atan2((point_b.y - point_a.y), (point_b.x - point_a.x))

def _min_angle_between_angles(angle_a, angle_b):
    # First, we normalize both angles so that we're working in the same bounded range
    angle_a = _normalize_rad(angle_a)
    angle_b = _normalize_rad(angle_b)

    # There are two other candidates for angle_a that we neeed to consider: the
    # immediately inferior and the immediately superior
    possible_angle_as = [angle_a - 2 * math.pi, angle_a, angle_a + 2 * math.pi]
    angle_differences = [angle_b - possible_angle_a for possible_angle_a in possible_angle_as]

    # The angle difference that we want is the one with the lowest absolute value
    min_abs_diff, idx = min((abs(diff), idx) for (idx, diff) in enumerate(angle_differences))
    return angle_differences[idx]

def _is_spin_clockwise(current_angle, target_angle):
    return _min_angle_between_angles(current_angle, target_angle) < 0

def _are_points_equal(point_a, point_b, tolerance):
    return math.sqrt((point_a.x - point_b.x) ** 2 + (point_a.y - point_b.y) ** 2) < tolerance

def _are_angles_equal(angle_a, angle_b, tolerance):
    return abs(_normalize_rad(angle_a) - _normalize_rad(angle_b)) < tolerance

def _normalize_rad(rad):
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad <= -math.pi:
        rad += 2 * math.pi
    return rad

def _deg_to_rad(deg):
    return math.pi * deg / 180

def _clamp(val, min_val, max_val):
    if min_val > max_val: raise ValueError
    return max(min_val, min(val, max_val))

class _TestHelperFunctions(unittest.TestCase):
    def test_clamp(self):
        self.assertEqual(_clamp(10, 1, 100), 10)
        self.assertEqual(_clamp(10, 20, 100), 20)
        self.assertEqual(_clamp(10, 1, 5), 5)
        with self.assertRaises(ValueError):
            _clamp(10, 1, 0)

    def test_deg_to_rad(self):
        self.assertEqual(_deg_to_rad(0), 0)

if __name__ == "__main__":
    print "Testing draw_figure: for normal usage, import the module"
    unittest.main()
