#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from fusion.msg import FusionMsg


def callback(data):
    print data

if __name__ == '__main__':
    rospy.init_node('classifier')
    rospy.Subscriber("/fusion/results", FusionMsg, callback)
    rospy.spin()
