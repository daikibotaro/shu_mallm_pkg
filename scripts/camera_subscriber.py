#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

cv_image = None

def callback(data):
    global cv_image
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
        rospy.signal_shutdown("Image received, shutting down subscriber node.")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def usb_camera_subscriber():
    global cv_image
    rospy.init_node('usb_camera_subscriber', anonymous=True)
    rospy.Subscriber('/usb_camera/image_raw', Image, callback)

    cv2.namedWindow("USB Camera", cv2.WINDOW_NORMAL)
    rospy.spin()

    if cv_image is not None:
        while True:
            screen_width = cv2.getWindowImageRect("USB Camera")[2]
            screen_height = cv2.getWindowImageRect("USB Camera")[3]
            resized_image = cv2.resize(cv_image, (screen_width, screen_height))
            cv2.imshow("USB Camera", resized_image)
            if cv2.waitKey(100) == 27:  # Press 'ESC' to exit the loop
                break
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        usb_camera_subscriber()
    except rospy.ROSInterruptException:
        pass

