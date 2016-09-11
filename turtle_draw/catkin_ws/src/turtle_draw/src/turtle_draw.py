#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose

from math import atan, pi

# The turtle's position - this is set by the subsriber callback
turtle_pose = Pose()

def main():
    rospy.init_node("turtle_draw")

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    rate = rospy.Rate(10) # 10 Hz

    move_straight(Pose(x=8, y=8), 1, 0.1, rate, pub)

    rospy.loginfo("We're done!")
    while not rospy.is_shutdown():
        rate.sleep()


def pose_callback(pose):
    global turtle_pose
    turtle_pose = pose

def move_straight(target, speed, tolerance, rate, pub):
    rospy.loginfo("Going to x: %.2f y: %.2f" % (target.x, target.y))

    # We first spin, then move forward

    target_theta = angle_between_points(target, turtle_pose)

    rospy.loginfo("Need to aquire theta: %.2f" % target_theta)
    while not are_angles_equal(turtle_pose.theta, target_theta, deg_to_rad(1)) and not rospy.is_shutdown():
        spin(speed, target_theta < 0, pub)
        rate.sleep()

    rospy.loginfo("Done spinning!")
    stop(pub) # Stop spinning

    rospy.loginfo("Moving to the target")
    while not are_points_equal(turtle_pose, target, 0.2, coords=["x", "y"]) and not rospy.is_shutdown():
        advance(speed, True, pub)
        rate.sleep()

    rospy.loginfo("Reached the target!")
    stop(pub) # Stop moving

def angle_between_points(point_a, point_b):
    if abs(point_b.x - point_a.x) < 0.0001: # These are floating point numbers, so we don't check for equality directly
        if point_b.y - point_a.y > 0:
            return pi/2
        else:
            return -pi/2
    else:
        return atan((point_b.y - point_a.y) / (point_b.x - point_a.x))

def are_points_equal(point_a, point_b, tolerance, coords=["x", "y", "z"]):
    return all([abs(getattr(point_a, coord) - getattr(point_b, coord)) < tolerance for coord in coords])

def are_angles_equal(angle_a, angle_b, tolerance):
    return abs(normalize_rad(angle_a) - normalize_rad(angle_b)) < tolerance

def spin(speed, clockwise, pub):
    pub.publish(Twist(linear=Point(0, 0, 0), angular=Point(0, 0, speed * (-1 if clockwise else 1))))

def advance(speed, forward, pub):
    pub.publish(Twist(linear=Point(speed * (1 if forward else -1), 0, 0), angular=Point(0, 0, 0)))

def stop(pub):
    advance(0, True, pub)

def deg_to_rad(deg):
    return pi * deg / 180

def normalize_rad(rad):
    while rad > pi:
        rad -= 2 * pi
    while rad <= -pi:
        rad += 2 * pi
    return rad

if __name__ == "__main__":
    main()
