#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose

from math import atan

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
    rospy.loginfo("x: %.2f y: %.2f" % (pose.x, pose.y))

def move_straight(target, speed, tolerance, rate, pub):
    rospy.loginfo("Going to x: %.2f y: %.2f" % (target.x, target.y))

    # We first spin, then move forward

    target_theta = Pose(theta=angle_between_points(target, turtle_pose))

    rospy.loginfo("Need to aquire theta: %.2f" % target_theta.theta)
    while not are_points_equal(turtle_pose, target_theta, 0.2, coords=["theta"]) and not rospy.is_shutdown():
        spin(speed, False, pub)
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
    return atan((point_b.y - point_a.y) / (point_b.x - point_a.x))

def are_points_equal(point_a, point_b, tolerance, coords=["x", "y", "z"]):
    return all([abs(getattr(point_a, coord) - getattr(point_b, coord)) < tolerance for coord in coords])

def spin(speed, clockwise, pub):
    pub.publish(Twist(linear=Point(0, 0, 0), angular=Point(0, 0, speed * (-1 if clockwise else 1))))

def advance(speed, forward, pub):
    pub.publish(Twist(linear=Point(speed * (1 if forward else -1), 0, 0), angular=Point(0, 0, 0)))

def stop(pub):
    advance(0, True, pub)

if __name__ == "__main__":
    main()
