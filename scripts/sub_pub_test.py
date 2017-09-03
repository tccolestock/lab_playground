#!/usr/bin/env python

"""
Attempting to create a template on how to subscribe to a topic and republish a calculated value based on the subscribed data.

BioRobotics Lab, Florida Atlantic University, 2017
"""

from __future__ import division
import rospy
from std_msgs.msg import String, Float32, UInt8
from turtlesim.msg import Pose


rospy.init_node("turtle_sub_pub", anonymous=True)


def listen():
    rospy.Subscriber("turtlesim/pose", Pose, callback)
    rospy.spin()


def callback(data):
    poseX = data.x
    talk(posex)


def talk(msg):
    pub_handle = rospy.Publisher("turtle_sub_pub", flaot32, queue_size=10)
    pub_handle.publish(msg * 2)


if __name__ == '__main__':
    try:
        listen()
    except ROSInterruptException:
        pass
