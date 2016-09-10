#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("logger", anonymous=True)
    rate = rospy.Rate(1) # 10 Hz

    while not rospy.is_shutdown():
        hello_str = "Hello world! Time: %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        rate.sleep()

if __name__ == "__main__":
    main()
