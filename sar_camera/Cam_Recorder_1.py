#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/stereo/left/image_rect_color/image_processed", Image, self.callback, queue_size=500)
        self.counter = 0
        os.makedirs("/tmp/Gazebo_Recording_1", exist_ok=True)


    def callback(self, data):
        try:
            # Convert the image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.t = int(data.header.stamp.to_time()*1000)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Save the image
        filename = f"/tmp/Gazebo_Recording_1/image_{self.counter:d}.png"
        cv2.imwrite(filename, cv_image)
        rospy.loginfo("Saved image {}".format(filename))
        self.counter += 1

def main():
    rospy.init_node('image_saver', anonymous=True)
    image_saver = ImageSaver()
    rospy.spin()

if __name__ == '__main__':
    main()
