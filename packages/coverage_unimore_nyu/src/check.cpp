#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
// #include "coverage_unimore_nyu/PoseVector.h"
#include <tf2/utils.h>

class Checker
{
private:
    ros::Publisher pub;
    ros::Subscriber vel_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber neighbor_sub;
    ros::NodeHandle n;
    ros::Timer timer;
    ros::Time begin;
    Eigen::Vector3d u_des, p_i;
    Eigen::MatrixXd p_j, p_j_old;
    int neighbors_count = 0;
    int neighbors_count_old = 0;
    double V_MAX = 0.5;                     // max robots speed
    double distance = 2.0;                  // min distance allowed between robots
    bool just_lost = true;
    double theta;
    int lost = 99;

public:
    Checker()
    {
        vel_sub = n.subscribe<geometry_msgs::TwistStamped> ("/cmd_vel", 1, &Checker::vel_callback, this);
        neighbor_sub = n.subscribe<geometry_msgs::PoseArray>("/supervisor/robot0/pose",1,&Checker::neigh_callback,this);
        pose_sub = n.subscribe<nav_msgs::Odometry>("/hummingbird1/ground_truth/odometry",1,&Checker::pose_callback,this);
        pub = n.advertise<geometry_msgs::TwistStamped> ("/hummingbird1/autopilot/velocity_command", 1);
        timer = n.createTimer(ros::Duration(0.1), &Checker::timerCallback,this);

        p_i << 0.0,0.0,0.0;
        p_j.resize(3,2);
        p_j_old.resize(3,2);
                
        p_j << 100.0,0.0,0.0,100.0,0.0,0.0;
        p_j_old = p_j;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        geometry_msgs::TwistStamped vel_msg;
        if (neighbors_count < neighbors_count_old){
            if(just_lost)
            {
                std::cout << "I LOST A NEIGHBOR" << std::endl;
                begin = ros::Time::now();
                theta = p_i(2);                 // save current theta
                just_lost = false;

                // find lost neighbor
                for (int i = 0; i < neighbors_count_old; i++)
                {
                    for (int j = 0; j < neighbors_count; j++)
                    {
                        double wdist = 2*V_MAX*ros::Duration(0.1).toSec();             // distanza percorsa a vmax in 0.1s
                        double d = (p_j_old(0,i) - p_j(0,j))*(p_j_old(0,i) - p_j(0,j)) + (p_j_old(1,i) - p_j(1,j))*(p_j_old(1,i) - p_j(1,j));
                        // std::cout << "Distance from " << i << " to " << j << " is " << d << std::endl;
                        // std::cout << "Worst case distance: " << wdist << std::endl;
                        if (d > wdist)
                        {
                            lost = i;
                            std::cout << "Lost robot: " << lost << std::endl;
                        }
                    }
                }
                if (lost == 99)
                {
                    std::cout << "CANNOT UNDERSTAND WHICH ONE IS LOST" << std::endl;
                }
                
            }
            double d_est = estimate_distance(begin, lost);
            std::cout << "Estimated distance: " << d_est << std::endl;
            if (d_est < distance){
                std::cout << "I MAY BE TOO CLOSE TO MY NEIGHBOR" << std::endl;
                double theta_des = M_PI + atan2(p_j_old(lost,1), p_j_old(lost,0));
                double w = 0.5 * (p_i(2) - theta_des);              // calculate angular velocity to make 360° turn
                u_des(2) = w;
                // std::cout << "Saved theta: " << theta << std::endl;
                // std::cout << "Current theta: " << p_i(2) << std::endl;
                // std::cout << "Calculated omega: " << w << std::endl;
            }
        } else
        {
            neighbors_count_old = neighbors_count;
            p_j_old = p_j;
            just_lost = true;
        }

        // std::cout << "Sending angular velocity: " << u_des(2) << std::endl;
        
        vel_msg.twist.linear.x = u_des(0);
        vel_msg.twist.linear.y = u_des(1);
        vel_msg.twist.angular.z = u_des(2);
        pub.publish(vel_msg);
    }

    void neigh_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
    {
        for (int i=0; i<msg->poses.size(); i++)
        {
            //ROS_INFO("GOT POSE %d",i);
            p_j(0,i) = msg->poses[i].position.x;
            p_j(1,i) = msg->poses[i].position.y;
            p_j(2,i) = tf2::getYaw(msg->poses[i].orientation);
        }

        neighbors_count = msg->poses.size();
        std::cout << "Number of neighbors: " << neighbors_count << std::endl;
    }
    
    void pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
        {
            p_i(0) = msg->pose.pose.position.x;
            p_i(1) = msg->pose.pose.position.y;
            p_i(2) = tf2::getYaw(msg->pose.pose.orientation);
        }

    

    void vel_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        u_des(0) = msg->twist.linear.x;
        u_des(1) = msg->twist.linear.y;
        u_des(2) = msg->twist.angular.z;
    }

    double estimate_distance(ros::Time t0, int l)
    {
        // estimate position of lost neighbor considering its max speed
        ros::Time tNow = ros::Time::now();


        double dx = (p_i(0) - p_j_old(l,0));
        double dy = (p_i(1) - p_j_old(l,1));
        double distance_est = std::sqrt(std::pow(dx,2) + std::pow(dy,2)) - V_MAX*((tNow - t0).toSec());
        return distance_est;
    }

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    ros::init(argc, argv, "front_end");
    Checker Check_obj;

    ros::spin();

    return 0;
}