#!/usr/bin/env python

import math
import unittest

class Pose():
    def __init__(self, x, y, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return "Pose (x: %.2f y: %.2f theta: %.2f)" % (self.x, self.y, self.theta)

    def __repr__(self):
        return str(self)

## Rquired dependencies (dictionary with functions):
##   log(s): Prints logging information
##   pause(): Returns true while simulation progress should be paused
##   abort(): Returns true when the simulation should stop
##   step(): Idles until the simulation is ready to continue (executed at the end of each step)
##   curr_pose(): Returns the current pose
##   move(linear_speed, angular_speed): Moves the object forward and makes it spin
##   pen(on): Turns the pen on or off
def draw(points, deps):
    # We need to turn of the pen while we reach the first point
    deps["pen"](False)
    _move_straight(Pose(x=points[0].x, y=points[0].y), 1, 1, 0.1, deps)

    # And then turn it back on to draw the different segments
    deps["pen"](True)
    for point in points[1:]:
        _move_straight(Pose(x=point.x, y=point.y), 1, 1, 0.1, deps)

    deps["log"]("Done!")

def _move_straight(target, move_speed, spin_speed, pos_tolerance, deps):
    deps["log"]("Going to x: %.2f y: %.2f" % (target.x, target.y))

    # We first spin, then move forward
    target_theta = _angle_between_points(deps["curr_pose"](), target)
    deps["log"]("Need to aquire theta: %.2f" % target_theta)

    while not _are_angles_equal(deps["curr_pose"]().theta, target_theta, _deg_to_rad(5)) and not deps["abort"]():
        _pause_if_required(deps)

        _spin(spin_speed, _is_spin_clockwise(deps["curr_pose"]().theta, target_theta), deps)
        deps["step"]()

    deps["log"]("Done spinning!")
    _stop(deps) # Stop spinning

    deps["log"]("Moving to the target")
    # We don't move in a straight line because we might need to do small angle corrections
    _move(target, move_speed, spin_speed, pos_tolerance, deps)

    deps["log"]("Reached the target!")
    _stop(deps) # Stop moving

def _move(target, move_speed, spin_speed, pos_tolerance, deps):
    while not _are_points_equal(deps["curr_pose"](), target, pos_tolerance) and not deps["abort"]():
        _pause_if_required(deps)

        # We move forward, but may need to apply small angle corrections while doing so
        target_theta = _angle_between_points(deps["curr_pose"](), target)
        angle_correction = _min_angle_between_angles(deps["curr_pose"]().theta, target_theta)

        # We could apply some proportional gain to angle_correction here, but too much
        # will make us zigzag
        correction_spin_speed = _clamp(angle_correction, -spin_speed, spin_speed)
        deps["move"](move_speed, correction_spin_speed)

        deps["step"]()

def _spin(speed, clockwise, deps):
    deps["move"](0, speed * (-1 if clockwise else 1))

def _stop(deps):
    deps["move"](0, 0)

def _pause_if_required(deps):
    if deps["pause"]():
        _stop(deps) # Halt
        while deps["pause"](): # Idle loop until the pause ends
            deps["step"]()

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
    # Normalization to (-pi, pi]
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
        self.assertAlmostEqual(_normalize_rad(0), 0) # No offset
        self.assertAlmostEqual(_normalize_rad(math.pi / 2), math.pi / 2) # No normalization needed (positive angle)
        self.assertAlmostEqual(_normalize_rad(-math.pi / 2), -math.pi / 2) # No normalization needed (negative angle)

        # Edge cases
        self.assertAlmostEqual(_normalize_rad(math.pi), math.pi) # pi is not normalized
        self.assertAlmostEqual(_normalize_rad(-math.pi), math.pi) # -pi is normalized

        # Extreme cases
        self.assertAlmostEqual(_normalize_rad(2 * math.pi), 0) # Large positive angle
        self.assertAlmostEqual(_normalize_rad(-2 * math.pi), 0) # Large negative angle
        self.assertAlmostEqual(_normalize_rad(20 * math.pi), 0) # Very large positive angle
        self.assertAlmostEqual(_normalize_rad(-20 * math.pi), 0) # Very large negative angle

    def test_deg_to_rad(self):
        # Normal usage
        self.assertAlmostEqual(_deg_to_rad(0), 0) # No offset
        self.assertAlmostEqual(_deg_to_rad(180), math.pi) # Normal case
        self.assertAlmostEqual(_deg_to_rad(-180), -math.pi) # Negative angles

        # Extreme cases
        self.assertAlmostEqual(_deg_to_rad(180 + 360), 3 * math.pi) # Angles over 360 degrees
        self.assertAlmostEqual(_deg_to_rad(-180 - 360), -3 * math.pi) # Negative Angles over 360 degrees

    def test_clamp(self):
        # Normal usage
        self.assertEqual(_clamp(10, 1, 100), 10) # No clamping
        self.assertEqual(_clamp(10, 20, 100), 20) # Clamping at bottom
        self.assertEqual(_clamp(10, 1, 5), 5) # Clamping at top
        self.assertEqual(_clamp(10, 5, 5), 5) # Clamping at both

        # Bad usage
        with self.assertRaises(ValueError):
            _clamp(10, 1, 0) # Max smaller than min

if __name__ == "__main__":
    print "Testing draw_figure: for normal usage, import the module"
    unittest.main()
