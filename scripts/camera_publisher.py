#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def usb_camera_publisher():
    rospy.init_node('usb_camera_publisher', anonymous=True)
    pub = rospy.Publisher('/usb_camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture('/dev/video0')

    if not cap.isOpened():
        rospy.logerr("Cannot open USB camera")
        return

    ret, frame = cap.read()
    if not ret:
        rospy.logerr("Cannot receive frame from USB camera")
        return

    try:
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(ros_image)
        rospy.loginfo("Published image")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    cap.release()


if __name__ == '__main__':
    try:
        usb_camera_publisher()
    except rospy.ROSInterruptException:
        pass

