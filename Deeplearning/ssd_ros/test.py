#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
from std_msgs.msg import String
import cv2
import sys

def main():

    try:

        nano_status_pub = rospy.Publisher('nano/status', String, queue_size=1)
        rospy.init_node('test_pub', anonymous=True)

        while not rospy.is_shutdown():

            image = cv2.imread('/home/michael/test_realsense/test/cat.jpg')
            cv2.imshow('Image', image)

            key = cv2.waitKey(1)
            if key == ord('a'):
                status_msg = 'person'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('b'):
                status_msg = 'red'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('c'):
                status_msg = 'green'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('d'):
                status_msg = 'no_right_turn'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('e'):
                status_msg = 'speed_up'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('f'):
                status_msg = 'school_zone'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('g'):
                status_msg = 'school_zone_off'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('h'):
                status_msg = 'roundabout'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('i'):
                status_msg = 'tunnel'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('j'):
                status_msg = 'first_parking'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('k'):
                status_msg = 'second_parking'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('l'):
                status_msg = 'left_turn'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('m'):
                status_msg = 'avoid_right'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)
            elif key == ord('n'):
                status_msg = 'avoid_left'
                rospy.loginfo("Publish the Nano Status")
                nano_status_pub.publish(status_msg)

    except KeyboardInterrupt:
        print("Shutting down")
        sys.exit()

    except rospy.ROSInterruptException:
        print("Shutting down")
        pass
        sys.exit()

    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    
        
