#!/usr/bin/env python
import rospy

from tf_conversions import transformations
from math import pi
import tf

class Robot:
    def __init__(self,robotID):
        self.map = robotID + "_tf/map"
        self.base_link = robotID + "_tf/base_link"
        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform(self.map, self.base_link, rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

    def get_pos(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.map, self.base_link, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return (None, None)

        return (trans, rot)