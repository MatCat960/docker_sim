#! /usr/bin/env python
import scipy as sp
from scipy.spatial.transform import Rotation
import numpy as np
import os

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

def get_homogeneous(rot, t):
    return np.vstack((np.hstack((rot, t.reshape(3, 1))), [0, 0, 0, 1]))

def split_homogeneous(T):
    return T[:3, :3], T[:3, 3]

def get_homogeneous_inverse(T):
    rot, t = split_homogeneous(T)
    Rinv = rot.transpose()
    tinv = -Rinv.dot(t)
    return get_homogeneous(Rinv, tinv)# np.linalg.inv(T)

def get_homogeneous_identity():
    return get_homogeneous(np.identity(3), np.zeros((3, 1)))

def odom_to_homogeneous(odom_msg):
    return get_homogeneous( Rotation.from_quat(np.array([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])).as_matrix(), 
                            np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z]))

def imu_to_homogeneous(imu_msg):
    return get_homogeneous( Rotation.from_quat(np.array([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])).as_matrix(), 
                            np.array([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z]))

def eul_trans_to_homogeneous(eul, tr):
    return get_homogeneous( Rotation.from_euler("ZYX", eul, degrees=True).as_matrix(), tr)

class VoxlLocalization:
    def __init__(self):
        self.initialized_Hwb = False
        self.initialized_Hob = False
        
        self.global_odom = Odometry()
        self.vio_odom = Odometry()

        self.Hwb = get_homogeneous_identity() # World to Body : VICON
        self.Hob = get_homogeneous_identity() # Odom to Camera : VIO
        self.Hwo = get_homogeneous_identity() # Tag to Camera : Tag inverse detection

        self.vio_global_odom_publisher = rospy.Publisher("coverage_odom", Odometry, queue_size=1)

        self.vicon_subscriber = rospy.Subscriber("odom", Odometry, self.vicon_odom_callback)
        self.vio_subscriber = rospy.Subscriber("qvio/odometry", Odometry, self.vio_odom_callback)

        rospy.spin()

    def vio_odom_callback(self, msg):
        self.Hob = odom_to_homogeneous(msg)
        if not self.initialized_Hob:
            self.initialized_Hob = True
        if self.initialized_Hwb:
            self.Hob = odom_to_homogeneous(msg)
            global_h = np.dot(self.Hwo, self.Hob)
            print("VIO - " + rospy.get_namespace())
            print(f"{self.Hob}")
            print("VIO + VICON Init: " + rospy.get_namespace())
            print(f"{global_h}")
            print("Vicon: " + rospy.get_namespace())
            print(f"{self.Hwb}")

            coverage_odom = Odometry()
            coverage_odom.header.stamp = rospy.Time.now()
            coverage_odom.header.frame_id = "world"
            rot, tr = split_homogeneous(global_h)
            quat = Rotation.from_matrix(rot).as_quat() 
            coverage_odom.pose.pose.position.x = tr[0] 
            coverage_odom.pose.pose.position.y = tr[1] 
            coverage_odom.pose.pose.position.z = tr[2]

            coverage_odom.pose.pose.orientation.x = quat[0] 
            coverage_odom.pose.pose.orientation.y = quat[1] 
            coverage_odom.pose.pose.orientation.z = quat[2] 
            coverage_odom.pose.pose.orientation.w = quat[3]

            self.vio_global_odom_publisher.publish(coverage_odom)
            
        return

    def vicon_odom_callback(self, msg):
        if not self.initialized_Hwb and self.initialized_Hob:
            print("1 False")
            self.Hwb = odom_to_homogeneous(msg)
            # print(f"{Hwb_vicon=}")
            self.Hwo = np.dot(self.Hwb, get_homogeneous_inverse(self.Hob))
            print("HWO")
            print(f"{self.Hwo}")
            self.initialized_Hwb = True
            return
        self.Hwb = odom_to_homogeneous(msg)
        return
    

rospy.init_node("localize_vio_tag")
vl = VoxlLocalization()
