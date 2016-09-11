#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose

from math import atan2, sqrt, pi
import time
import argparse

# The turtle's position - this is set by the subsriber callback
turtle_pose = Pose()

def main():
    points = parse_cmd_args()

    rospy.init_node("turtle_draw")

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    rate = rospy.Rate(10) # 10 Hz

    time.sleep(0.5) # We need to sleep for a bit to let the subscriber fetch the current pose at least once

    for point in points:
        move_straight(Pose(x=point[0], y=point[1]), 1, 1, 0.1, rate, pub)

    rospy.loginfo("We're done!")
    while not rospy.is_shutdown():
        rate.sleep()

def parse_cmd_args():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--figure", help="a figure file")
    parser.add_argument("--reverse", help="draw the figure in reverse order", default=False, action="store_true")
    args = vars(parser.parse_args())

    if args["figure"]:
        points = parse_figure_file(args["figure"])
    else: # Use the default figure
        points = [[8, 8], [5, 10], [2, 8], [0, 5], [2, 2], [5, 0], [8, 2], [10, 5], [8, 8]]

    if args["reverse"]:
        points.reverse()

    return points

def parse_figure_file(filename):
    with open(filename, "r") as f:
        lines = f.readlines()

    points = []
    for line in lines:
        points.append([float(i) for i in line.split(",")])
    return points

def pose_callback(pose):
    global turtle_pose
    turtle_pose = pose

def move_straight(target, move_speed, spin_speed, pos_tolerance, rate, pub):
    rospy.loginfo("Going to x: %.2f y: %.2f" % (target.x, target.y))

    # We first spin, then move forward

    target_theta = angle_between_points(turtle_pose, target)

    rospy.loginfo("Need to aquire theta: %.2f" % target_theta)
    while not are_angles_equal(turtle_pose.theta, target_theta, deg_to_rad(5)) and not rospy.is_shutdown():
        spin(spin_speed, is_spin_clockwise(turtle_pose.theta, target_theta), pub)
        rate.sleep()

    rospy.loginfo("Done spinning!")
    stop(pub) # Stop spinning

    rospy.loginfo("Moving to the target")
    # We don't move in a straight line because we might need to do small angle corrections
    move(target, move_speed, spin_speed, pos_tolerance, rate, pub)

    rospy.loginfo("Reached the target!")
    stop(pub) # Stop moving

def move(target, move_speed, spin_speed, pos_tolerance, rate, pub):
    while not are_points_equal(turtle_pose, target, pos_tolerance) and not rospy.is_shutdown():
        # We move forward, but may need to apply small angle corrections while doing so
        target_theta = angle_between_points(turtle_pose, target)
        angle_correction = min_angle_between_angles(turtle_pose.theta, target_theta)

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
