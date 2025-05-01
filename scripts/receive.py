#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def print_robot_position():
    rospy.init_node('robot_position_listener', anonymous=True)
    
    # Wait for one message from the topic and store it
    pose_msg = rospy.wait_for_message('/robot_map_position', PoseStamped)
    
    # Print the received message
    print("Received PoseStamped message:")
    print("Position:")
    print("  x: {}".format(pose_msg.pose.position.x))
    print("  y: {}".format(pose_msg.pose.position.y))
    print("  z: {}".format(pose_msg.pose.position.z))
    print("Orientation:")
    print("  x: {}".format(pose_msg.pose.orientation.x))
    print("  y: {}".format(pose_msg.pose.orientation.y))
    print("  z: {}".format(pose_msg.pose.orientation.z))
    print("  w: {}".format(pose_msg.pose.orientation.w))

if __name__ == '__main__':
    try:
        print_robot_position()
    except rospy.ROSInterruptException:
        pass

