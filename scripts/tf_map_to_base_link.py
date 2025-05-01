#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped

def publish_robot_position():
    rospy.init_node('robot_position_publisher')

    listener = tf.TransformListener()
    position_pub = rospy.Publisher('/robot_map_position', PoseStamped, queue_size=10)

    rate = rospy.Rate(10.0)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform('map', 'base_link', rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))

            # Create a PoseStamped message
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]

            # Publish the PoseStamped message
            position_pub.publish(pose)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF Exception occurred")

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_robot_position()
    except rospy.ROSInterruptException:
        pass

