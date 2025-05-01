import rospy
from geometry_msgs.msg import PoseStamped
import math


def get_robot_position():
    rospy.init_node('robot_position_listener', anonymous=True)

    # Wait for one message from the topic and store it
    pose_msg = rospy.wait_for_message('/robot_map_position', PoseStamped)

    # Print the received message
    x, y = pose_msg.pose.position.x, pose_msg.pose.position.y

    z, w = pose_msg.pose.orientation.z, pose_msg.pose.orientation.w
    theta = math.atan2(z, w) * 360 / math.pi

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

    print(f"\ntheta: {theta}")

    return x, y, theta


if __name__ == '__main__':
    try:
        get_robot_position()
    except rospy.ROSInterruptException:
        pass
