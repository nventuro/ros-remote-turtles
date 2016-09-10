#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose

turtle_pose = Pose()

def main():
    rospy.init_node("turtle_draw")

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        t = Twist(linear = Point(2 , 2, 0), angular = Point(0, 0, 5))
        pub.publish(t)

        rospy.loginfo("pose = x: %d y: %d theta: %d" % (turtle_pose.x, turtle_pose.y, turtle_pose.theta))

        rate.sleep()

def pose_callback(pose):
    global turtle_pose
    turtle_pose = pose

if __name__ == "__main__":
    main()
