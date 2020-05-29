#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import cv2

bridge = CvBridge()

def raw_image_callback(data):

    try:
        image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow('raw_image_sub', image)
    rospy.loginfo("Subscribe the Raw Image")
    cv2.waitKey(1)

def compressed_image_callback(data):

    np_arr = np.fromstring(data.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imshow('compressed_image_sub', image)
    rospy.loginfo("Subscribe the Compressed Image")
    cv2.waitKey(1)

def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('image_sub', anonymous=True)

    # Sub Raw Image
    rospy.Subscriber("camera/raw_image", Image, raw_image_callback)

    # Sub Compressed Image
    rospy.Subscriber("camera/compressed_image", CompressedImage, compressed_image_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
