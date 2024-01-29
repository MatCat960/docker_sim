#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <fow_control/FowController.h>
#include <tf2/utils.h>

class Subscribe_And_Publish
{
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    std::vector<ros::Subscriber> pose_subs;
    ros::NodeHandle n;
    ros::Timer timer;
    fow_control::FowController controller;
    Eigen::Vector3d ustar,p_i;
    Eigen::VectorXd h;
    Eigen::MatrixXd p_j;

public:


    Subscribe_And_Publish(): controller(1.7453,0.750,10.0,3)
    {
        sub = n.subscribe<geometry_msgs::Twist> ("/cmd_vel_in", 1, &Subscribe_And_Publish::vel_callback, this);
        pose_subs.push_back(n.subscribe<nav_msgs::Odometry>("/pose1",1,[this](const nav_msgs::Odometry::ConstPtr &msg){this->pose_callback(msg,1);}));
        pose_subs.push_back(n.subscribe<nav_msgs::Odometry>("/pose2",1,[this](const nav_msgs::Odometry::ConstPtr &msg){this->pose_callback(msg,2);}));
        pose_subs.push_back(n.subscribe<nav_msgs::Odometry>("/pose3",1,[this](const nav_msgs::Odometry::ConstPtr &msg){this->pose_callback(msg,3);}));
        pub = n.advertise<geometry_msgs::TwistStamped> ("/cmd_vel_out", 1);
        timer = n.createTimer(ros::Duration(0.1), &Subscribe_And_Publish::timerCallback,this);
        
        p_i << 0.0,0.0,0.0;
        p_j.resize(3,2);
                
        p_j << 100.0,0.0,0.0,100.0,0.0,0.0;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        Eigen::Vector3d uopt;
        geometry_msgs::TwistStamped vel_msg;
        if(!controller.applyCbf(uopt,h,ustar,p_i,2,p_j))
        {        
            vel_msg.header.stamp = ros::Time::now();
            vel_msg.twist.linear.x = uopt(0);
            vel_msg.twist.linear.y = uopt(1);
            vel_msg.twist.angular.z = uopt(2);
            pub.publish(vel_msg);
            ROS_INFO("CBF SUCCESS");
        }
        else{
            ROS_INFO("CBF FAILED");
            ROS_ERROR("CBF FAILED");
        }
    }

    void pose_callback(const nav_msgs::Odometry::ConstPtr &msg,int i){
        if (i == 1){
            //ROS_INFO("GOT POSE 1");
            p_i(0) = msg->pose.pose.position.x;
            p_i(1) = msg->pose.pose.position.y;
            p_i(2) = tf2::getYaw(msg->pose.pose.orientation);
        }
        else{
            //ROS_INFO("GOT POSE %d",i);
            p_j(0,i-2) = msg->pose.pose.position.x;
            p_j(1,i-2) = msg->pose.pose.position.y;
            p_j(2,i-2) = tf2::getYaw(msg->pose.pose.orientation);
        }
    }

    void vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        ustar(0) = msg->linear.x;
        ustar(1) = msg->linear.y;
        ustar(2) = msg->angular.z;
    }

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    ros::init(argc, argv, "front_end");
    Subscribe_And_Publish SAPObject;

    ros::spin();

    return 0;
}
