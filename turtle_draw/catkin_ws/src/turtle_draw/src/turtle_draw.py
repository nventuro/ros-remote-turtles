#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn, SetPen

from math import atan2, sqrt, pi
from functools import partial
from random import random
import time
import argparse
import threading

# Each turtl's position - this is set by the subsriber callbacks
turtle_poses = {}

paused = False

def main():
    points_array, remote = parse_cmd_args()

    rospy.init_node("turtle_draw")

    # Reset turtlesim
    rospy.ServiceProxy("/reset", Empty)()

    # Kill the original turtle - we will later create new turtles for each thread
    rospy.ServiceProxy("/kill", Kill)("turtle1")

    if remote:
        rospy.Subscriber("/turtle_draw/run", Bool, run_callback)
        pause()

    threads = []
    for points in points_array:
        # Spawn a new turtle with random position and orientation
        turtle_name = rospy.ServiceProxy("/spawn", Spawn)(random() * 11, random() * 11, random() * 2 * pi, "").name
        t = threading.Thread(target=draw_figure, args = (points, turtle_name))
        threads.append(t)

    # Launch all threads
    for t in threads:
        t.start()

    # And wait for them to finish
    for t in threads:
        t.join()

    rospy.loginfo("We're done!")

def run_callback(run):
    # If the new message matches our current state, do nothing
    if run.data and paused:
        unpause()
    elif not run.data and not paused:
        pause()

def pause():
    rospy.loginfo("Pausing")
    global paused
    paused = True

def unpause():
    rospy.loginfo("Unpausing")
    global paused
    paused = False

def pause_if_required(pub):
    if paused:
        stop(pub)

        while paused:
            time.sleep(0)

def draw_figure(points, turtle_name):
    rate = rospy.Rate(10) # 10 Hz

    pub = rospy.Publisher("/%s/cmd_vel" % turtle_name, Twist, queue_size=10)
    rospy.Subscriber("/%s/pose" % turtle_name, Pose, partial(pose_callback, turtle_name=turtle_name))
    set_pen = rospy.ServiceProxy("/%s/set_pen" % turtle_name, SetPen)

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

def pen_off(pen):
    pen(255, 255, 255, 3, 1)

def pen_on(pen):
    # White, width of 3
    pen(255, 255, 255, 3, 0)

def parse_cmd_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--figure", help="a figure file")
    parser.add_argument("--reverse", help="draw the figure in reverse order", default=False, action="store_true")
    parser.add_argument("--remote", help="listen to /turtle_draw/run for run/pause messages", default=False, action="store_true")

    multi_opts = parser.add_mutually_exclusive_group()
    multi_opts.add_argument("--dual-turtles", help="draw the figure using two turtles", action="store_true")
    multi_opts.add_argument("--turtle-army", help="draw the figure using one turtle per line", action="store_true")

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

    # We need to return a list of lists of points, where each turtle will draw
    # lines between the points in its list.

    if args["dual_turtles"]: # Two turtles
        points_array = [
                        points[0 : (len(points) / 2) + 1], # The first turtle needs to go to the first point of the second turtle to complete the shape
                        points[len(points) / 2 : ]
                       ]
    elif args["turtle_army"]: # One turtle per segment
        points_array = [[points[n], points[n + 1]] for n in range(len(points) - 1)]
    else: # A single turtle draws the whol efigure
        points_array = [points]

    return points_array, args["remote"]

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
    rospy.loginfo("[%s] Going to x: %.2f y: %.2f" % (turtle_name, target.x, target.y))

    # We first spin, then move forward

    target_theta = angle_between_points(turtle_poses[turtle_name], target)

    rospy.loginfo("[%s] Need to aquire theta: %.2f" % (turtle_name, target_theta))
    while not are_angles_equal(turtle_poses[turtle_name].theta, target_theta, deg_to_rad(5)) and not rospy.is_shutdown():
        pause_if_required(pub)

        spin(spin_speed, is_spin_clockwise(turtle_poses[turtle_name].theta, target_theta), pub)
        rate.sleep()

    rospy.loginfo("[%s] Done spinning!" % turtle_name)
    stop(pub) # Stop spinning

    rospy.loginfo("[%s] Moving to the target" % turtle_name)
    # We don't move in a straight line because we might need to do small angle corrections
    move(target, move_speed, spin_speed, pos_tolerance, rate, pub, turtle_name)

    rospy.loginfo("[%s] Reached the target!" % turtle_name)
    stop(pub) # Stop moving

def move(target, move_speed, spin_speed, pos_tolerance, rate, pub, turtle_name):
    while not are_points_equal(turtle_poses[turtle_name], target, pos_tolerance) and not rospy.is_shutdown():
        pause_if_required(pub)

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
