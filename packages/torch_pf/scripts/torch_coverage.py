#!/usr/bin/env python3

import os, sys
import time

import torch

# ROS imports
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist, TwistStamped
from nav_msgs.msg import Odometry

# Custom imports
from hqp import Hqp
from fow_control import FowController


SAFETY_DIST = 2.0
MAX_LIN_VEL = 1.5
MAX_ANG_VEL = 1.5
CONVERGENCE_TOLERANCE = 0.1



class Controller():
    def __init__(self):
        # Get params from launch file
        self.ROBOTS_NUM = rospy.get_param("~ROBOTS_NUM")
        self.ROBOT_ID = rospy.get_param("~ROBOT_ID")
        self.ROBOT_RANGE = rospy.get_param("~ROBOT_RANGE")
        self.ROBOT_FOV = rospy.get_param("~ROBOT_FOV")
        self.AREA_SIZE_x = rospy.get_param("~AREA_SIZE_x")
        self.AREA_SIZE_y = rospy.get_param("~AREA_SIZE_y")
        self.AREA_BOTTOM = rospy.get_param("~AREA_BOTTOM")
        self.AREA_LEFT = rospy.get_param("~AREA_LEFT")
        self.GAUSS_X = rospy.get_param("~GAUSS_X")
        self.GAUSS_Y = rospy.get_param("~GAUSS_Y")
        self.SAVE_LOGS = rospy.get_param("~SAVE_LOGS")
        self.SAVE_CPU_TIME = rospy.get_param("~SAVE_CPU_TIME")

        # Initialize ROS pubs and subs
        for i in range(self.ROBOTS_NUM):
            rospy.Subscriber('/hummingbird{}/autopilot/odometry'.format(i+1), Odometry, self.realposeCallback, i)

        self.odomSub = rospy.Subscriber('/hummingbird{}/autopilot/odometry'.format(self.ROBOT_ID), Odometry, self.odomCallback)
        self.neighSub = rospy.Subscriber('supervisor/robot{}/pose'.format(self.ROBOT_ID), PoseArray, self.neighCallback)
        self.velPub = rospy.Publisher('/hummingbird{}/autopilot/velocity_command'.format(self.ROBOT_ID), TwistStamped, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.25), self.timerCallback)

        print("CIAOOOOO")



def main():
    model = torch.jit.load('/home/mattia/pf-training/SerializedModels/coverage_model.pt')
    model.eval()
    print("Model loaded:")
    print(model)


if __name__ == "__main__":
    rospy.init_node('torch_coverage')
    joy_to_twist = Controller()
    rospy.spin()