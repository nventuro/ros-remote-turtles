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

import draw_figure

# Each turtle's position - this is set by the subsriber callbacks
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

        # Get its pose
        subscribe_to_pose_updates(turtle_name)

        # Generate dependencies
        dependencies = {
            "log"       : lambda s: rospy.loginfo("[%s] %s" % (turtle_name, s)),
            "pause"     : lambda: paused,
            "abort"     : rospy.is_shutdown,
            "step"      : rospy.Rate(10).sleep, # 10Hz update rate
            "curr_pose" : partial(get_turtle_pose, turtle_name),
            "move"      : partial(move_turtle, pub=rospy.Publisher("/%s/cmd_vel" % turtle_name, Twist, queue_size=10)),
            "pen"       : partial(set_pen_on_off, set_pen=rospy.ServiceProxy("/%s/set_pen" % turtle_name, SetPen))
        }

        dependencies["pen"](True)

        t = threading.Thread(target=draw_figure.draw, args = (points, dependencies))
        threads.append(t)

    # Launch all threads
    for t in threads:
        t.start()

    # And wait for them to finish
    for t in threads:
        t.join()

    rospy.loginfo("We're done!")

def subscribe_to_pose_updates(turtle_name):
    rospy.Subscriber("/%s/pose" % turtle_name, Pose, partial(new_pose_callback, turtle_name=turtle_name))

    while turtle_name not in turtle_poses: # We need to sleep for a bit to let the subscriber fetch the current pose at least once
        time.sleep(0)

def new_pose_callback(pose, turtle_name):
    global turtle_poses
    turtle_poses[turtle_name] = pose

def get_turtle_pose(turtle_name):
    pose = turtle_poses[turtle_name]
    return draw_figure.Pose(pose.x, pose.y, pose.theta)

def move_turtle(linear_speed, angular_speed, pub):
    pub.publish(Twist(linear=Point(linear_speed, 0, 0), angular=Point(0, 0, angular_speed)))

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

def set_pen_on_off(on, set_pen):
    if on:
        set_pen(255, 255, 255, 3, 0) # On, white, width of 3
    else:
        set_pen(255, 255, 255, 3, 1) # Off

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

if __name__ == "__main__":
    main()
