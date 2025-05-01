#!/usr/bin/env python3

import rospy
import tf

rospy.init_node('tf_listener')
listener = tf.TransformListener()

try:
    listener.waitForTransform('map', 'base_link', rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
    print("Translation: ", trans)
    print("Rotation: ", rot)
except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    pass
