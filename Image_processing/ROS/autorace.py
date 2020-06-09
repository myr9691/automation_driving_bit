import rospy
import numpy as np
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import cv2


class ControlLane():
    def __init__(self):
        self.sub_lane = rospy.Subscriber('pi/lane', Int32, self.cbFollowLane, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

        self.lastError = 0
        self.MAX_VEL = 0.05

        rospy.on_shutdown(self.fnShutDown)

    def cbFollowLane(self, desired_center):
        center = desired_center.data

        error = center - 320

        Kp = 0.0025
        Kd = 0.02
        

        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error
        twist = Twist()
        twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 320) ** 2.2), 0.2)
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -max(angular_z, -0.5) if angular_z < 0 else -min(angular_z, 0.5)
        self.pub_cmd_vel.publish(twist)

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_lane')
    node = ControlLane()
    node.main()
