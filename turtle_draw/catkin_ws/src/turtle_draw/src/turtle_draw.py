#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

from math import atan2, sqrt, pi
from functools import partial
import time
import argparse

# Each turtl's position - this is set by the subsriber callbacks
turtle_poses = {}

def main():
    rospy.init_node("turtle_draw")

    points = parse_cmd_args()

    draw_figure(points, "turtle1")

def draw_figure(points, turtle_name):
    rate = rospy.Rate(10) # 10 Hz

    pub = rospy.Publisher("/%s/cmd_vel" % turtle_name, Twist, queue_size=10)
    rospy.Subscriber("/%s/pose" % turtle_name, Pose, partial(pose_callback, turtle_name=turtle_name))
    set_pen = rospy.ServiceProxy("%s/set_pen" % turtle_name, SetPen)

    time.sleep(0.5) # We need to sleep for a bit to let the subscriber fetch the current pose at least once

    pen_off(set_pen)
    move_straight(Pose(x=points[0].x, y=points[0].y), 1, 1, 0.1, rate, pub, turtle_name)

    pen_on(set_pen)
    for point in points[1:]:
        move_straight(Pose(x=point.x, y=point.y), 1, 1, 0.1, rate, pub, turtle_name)

    rospy.loginfo("We're done!")
    while not rospy.is_shutdown():
        rate.sleep()

def pen_off(pen):
    pen(255, 255, 255, 3, 1)

def pen_on(pen):
    # White, width of 3
    pen(255, 255, 255, 3, 0)

def parse_cmd_args():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--figure", help="a figure file")
    parser.add_argument("--reverse", help="draw the figure in reverse order", default=False, action="store_true")
    args = vars(parser.parse_args())

    if args["figure"]:
        coords = parse_figure_file(args["figure"])
    else: # Use the default figure
        coords = [[0.75, 0.75], [-0.75, 0.75], [-0.75, -0.75], [0.75, -0.75], [0.75, 0.75]]

    if args["reverse"]:
        coords.reverse()

    points = []
    for coord in coords:
        points.append(Point(x=transform_coord(coord[0]), y=transform_coord(coord[1])))

    return points

def parse_figure_file(filename):
    with open(filename, "r") as f:
        lines = f.readlines()

    coords = []
    for line in lines:
        coords.append([float(i) for i in line.split(",")])
    return coords

def transform_coord(coord):
    # The coords go from (-1, -1) to (1, 1): we need to translate and scale them
    # to turtlesim's coordinate system, which goes from (0, 0) to (11, 11)
    return ((coord + 1) / 2) * 11

def pose_callback(pose, turtle_name):
    global turtle_poses
    turtle_poses[turtle_name] = pose

def move_straight(target, move_speed, spin_speed, pos_tolerance, rate, pub, turtle_name):
    rospy.loginfo("Going to x: %.2f y: %.2f" % (target.x, target.y))

    # We first spin, then move forward

    target_theta = angle_between_points(turtle_poses[turtle_name], target)

    rospy.loginfo("Need to aquire theta: %.2f" % target_theta)
    while not are_angles_equal(turtle_poses[turtle_name].theta, target_theta, deg_to_rad(5)) and not rospy.is_shutdown():
        spin(spin_speed, is_spin_clockwise(turtle_poses[turtle_name].theta, target_theta), pub)
        rate.sleep()

    rospy.loginfo("Done spinning!")
    stop(pub) # Stop spinning

    rospy.loginfo("Moving to the target")
    # We don't move in a straight line because we might need to do small angle corrections
    move(target, move_speed, spin_speed, pos_tolerance, rate, pub, turtle_name)

    rospy.loginfo("Reached the target!")
    stop(pub) # Stop moving

def move(target, move_speed, spin_speed, pos_tolerance, rate, pub, turtle_name):
    while not are_points_equal(turtle_poses[turtle_name], target, pos_tolerance) and not rospy.is_shutdown():
        # We move forward, but may need to apply small angle corrections while doing so
        target_theta = angle_between_points(turtle_poses[turtle_name], target)
        angle_correction = min_angle_between_angles(turtle_poses[turtle_name].theta, target_theta)

        # We could apply some proportional gain to angle_correction here, but too much
        # will make us zigzag
        target_spin_speed = clamp(angle_correction, -spin_speed, spin_speed)

        pub.publish(Twist(linear=Point(move_speed, 0, 0), angular=Point(0, 0, target_spin_speed)))
        rate.sleep()

def spin(speed, clockwise, pub):
    pub.publish(Twist(linear=Point(0, 0, 0), angular=Point(0, 0, speed * (-1 if clockwise else 1))))

def stop(pub):
    pub.publish(Twist(linear=Point(0, 0, 0), angular=Point(0, 0, 0)))

def angle_between_points(point_a, point_b):
    # atan2 works correctly on all 4 quadrants, and we don't need to
    # worry about dividing by zero when the x coordinates match
    return atan2((point_b.y - point_a.y), (point_b.x - point_a.x))

def min_angle_between_angles(angle_a, angle_b):
    # First, we normalize both angles so that we're working in the same bounded range
    angle_a = normalize_rad(angle_a)
    angle_b = normalize_rad(angle_b)

    # There are two other candidates for angle_a that we neeed to consider: the
    # immediately inferior and the immediately superior
    possible_angle_as = [angle_a - 2 * pi, angle_a, angle_a + 2 * pi]
    angle_differences = [angle_b - possible_angle_a for possible_angle_a in possible_angle_as]

    # The angle difference that we want is the one with the lowest absolute value
    min_abs_diff, idx = min((abs(diff), idx) for (idx, diff) in enumerate(angle_differences))
    return angle_differences[idx]

def is_spin_clockwise(current_angle, target_angle):
    return min_angle_between_angles(current_angle, target_angle) < 0

def are_points_equal(point_a, point_b, tolerance):
    return sqrt((point_a.x - point_b.x) ** 2 + (point_a.y - point_b.y) ** 2) < tolerance

def are_angles_equal(angle_a, angle_b, tolerance):
    return abs(normalize_rad(angle_a) - normalize_rad(angle_b)) < tolerance

def deg_to_rad(deg):
    return pi * deg / 180

def normalize_rad(rad):
    while rad > pi:
        rad -= 2 * pi
    while rad <= -pi:
        rad += 2 * pi
    return rad

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

if __name__ == "__main__":
    main()
