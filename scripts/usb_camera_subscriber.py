#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import tkinter as tk


def get_screen_resolution():
    root = tk.Tk()
    width = root.winfo_screenwidth()
    height = root.winfo_screenheight()
    root.destroy()
    return width, height


def callback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        rotated_frame = cv2.rotate(cv_image, cv2.ROTATE_180)
        screen_width, screen_height = get_screen_resolution()
        resized_image = cv2.resize(rotated_frame, (screen_width, screen_height))
        cv2.imshow("USB Camera", resized_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


def usb_camera_subscriber():
    rospy.init_node('usb_camera_subscriber', anonymous=True)
    rospy.Subscriber('/usb_camera/image_raw', Image, callback)

    # cv2.namedWindow("USB Camera", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("USB Camera", 1920, 1080)
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        usb_camera_subscriber()
    except rospy.ROSInterruptException:
        pass
