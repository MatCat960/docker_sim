#! /usr/bin/env python
import time
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from trackers_msgs.msg import VelocityTrackerActionGoal
from trackers_msgs.srv import Transition, TransitionRequest, TransitionResponse

class VoxlCoverageCommander:
    def __init__(self):
        self.twist_command = TwistStamped()
        self.velocity_command = VelocityTrackerActionGoal()

        self.pos_mode = "std_trackers/LineTrackerMinJerkAction"
        self.vel_mode = "std_trackers/VelocityTrackerAction"
        self.current_mode = 0
        
        self.velocity_goal_publisher = rospy.Publisher("trackers_manager/velocity_tracker/VelocityTrackerAction/goal", VelocityTrackerActionGoal, queue_size=1)

        self.cmd_vel_sub = rospy.Subscriber("coverage_cmd_vel", TwistStamped, self.velocity_command_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        time.sleep(5)
        transition_service_name = 'trackers_manager/transition'
        rospy.wait_for_service(transition_service_name)
        velocity_transition_service = rospy.ServiceProxy(transition_service_name, Transition)
        velocity_transition_request = TransitionRequest("std_trackers/VelocityTrackerAction")
        try:
            resp = velocity_transition_service(velocity_transition_request)
            print(resp)
        except rospy.ServiceException as ex:
            print("Service error")
            exit(-1)


        rospy.spin()

    def velocity_command_callback(self, msg):
        self.twist_command = msg    

    def odom_callback(self, msg):
        dt_prev_command = msg.header.stamp.to_sec() - self.twist_command.header.stamp.to_sec()
        self.velocity_command.header = msg.header
        if(dt_prev_command > 1.0):
            self.velocity_command.goal.vx = 0.0
            self.velocity_command.goal.vy = 0.0
            self.velocity_command.goal.vz = 0.0
            self.velocity_command.goal.vyaw = 0.0
        else:
            self.velocity_command.goal.vx = self.twist_command.twist.linear.x
            self.velocity_command.goal.vy = self.twist_command.twist.linear.y
            self.velocity_command.goal.vz = self.twist_command.twist.linear.z
            self.velocity_command.goal.vyaw = self.twist_command.twist.angular.z
        
        self.velocity_goal_publisher.publish(self.velocity_command)

rospy.init_node("voxl_coverage_commander")
vcc = VoxlCoverageCommander()


            
