#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy

class JoyToTwistStamped:
    def __init__(self):
        self.pub = rospy.Publisher('/hummingbird1/autopilot/velocity_command', Twist, queue_size=1)
        self.sub = rospy.Subscriber('joy', Joy, self.callback)
        self.twist = Twist()
        # self.twist.header.frame_id = "/hummingbird1/base_link"

    def callback(self, data):
        # self.twist.header.stamp = rospy.Time.now()
        self.twist.linear.x = data.axes[1]
        self.twist.linear.y = data.axes[0]
        self.twist.linear.z = data.axes[4]
        self.twist.angular.z = data.axes[3]
        self.pub.publish(self.twist)


if __name__ == '__main__':
    rospy.init_node('joy2vel')
    joy_to_twist = JoyToTwistStamped()
    rospy.spin()