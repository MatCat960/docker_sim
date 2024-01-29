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

// My includes
#include "pid_control/PIDController.h"

class HeadingController
{
    private:
        ros::Publisher pub;
        ros::Publisher avg_pub;
        ros::Subscriber vel_sub;
        ros::Subscriber pose_sub;
        ros::Subscriber neighbor_sub;
        std::vector<ros::Subscriber> fake_super_sub;
        ros::NodeHandle n;
        ros::Timer timer;
        ros::Time begin;
        PID::PIDController controller;
        Eigen::Vector3d p_i, u_des;
        Eigen::MatrixXd p_j, p_j_old;
        Eigen::VectorXd d_i_j;                      // vector of distances between robot i and j
        int neighbors_count = 0;
        int neighbors_count_old = 0;
        double V_MAX = 0.5;                             // max robots linear speed
        double W_MAX = 1.0;                             // max robots angular speed
        double safety_distance = 2.0;                  // min distance allowed between robots
        bool just_lost = true;
        double theta;
        int lost = 99;
        int NEIGHBORS_NUM = 5;                          // neighbors = ROBOTS_NUM - 1
        double ROBOT_RANGE = 8.0;                            // sensing range
        double wd = V_MAX*ros::Duration(0.1).toSec();                    // distance traveled at VMAX in 0.1 seconds (loop rate)
        Eigen::VectorXd theta_left;
        // double theta_left = 2*M_PI;

    public:
        HeadingController(): controller(0.1, W_MAX, -W_MAX, 0.5, 0.0, 0.5)
        {
            vel_sub = n.subscribe<geometry_msgs::Twist> ("/cmd_vel", 1, &HeadingController::vel_callback, this);
            neighbor_sub = n.subscribe<geometry_msgs::PoseArray>("/supervisor/robot0/pose",1,&HeadingController::neigh_callback,this);
            pose_sub = n.subscribe<nav_msgs::Odometry>("/hummingbird1/ground_truth/odometry",1,&HeadingController::pose_callback,this);
            pub = n.advertise<geometry_msgs::TwistStamped> ("/hummingbird1/autopilot/velocity_command", 1);
            timer = n.createTimer(ros::Duration(0.1), &HeadingController::timerCallback,this);
            avg_pub = n.advertise<geometry_msgs::PoseStamped> ("avg_point", 1);
            
            p_i << 0.0,0.0,0.0;
            // p_j.resize(3,2);
            p_j.resize(3,5);
            // p_j << 100.0,0.0,0.0,100.0,0.0,0.0;
            p_j << 100.0,0.0,0.0,100.0,0.0,0.0,100.0,0.0,0.0,100.0,0.0,0.0,100.0,0.0,0.0;
            
            d_i_j.resize(NEIGHBORS_NUM);
            d_i_j = 100.0*Eigen::VectorXd::Ones(NEIGHBORS_NUM);

            theta_left.resize(NEIGHBORS_NUM);
            theta_left = 2*M_PI*Eigen::VectorXd::Ones(NEIGHBORS_NUM);

            std::cout << "Initialized." << std::endl;
        
        }


        void timerCallback(const ros::TimerEvent&)
        {
            double w = 0.0;                             // angular velocity contribution
            for (int i=0; i<d_i_j.size(); i++)
            {
                if (d_i_j(i) < safety_distance)
                {
                    std::cout << "======= WARNING ======= " << std::endl;
                    std::cout << "Collision imminent!" << std::endl;
                    std::cout << "Spinning..." << std::endl;
                    if (theta_left(i) > 0.0)
                    {
                        // make 360 degrees rotation
                        w += 1.0;
                        theta_left(i) -= w*ros::Duration(0.1).toSec();
                        std::cout << "Theta left: " << theta_left(i) << std::endl;
                    } else
                    {
                        // Robot not found even after spinning: consider it at distance = sensing range
                        std::cout << "Spinning procedure completed." << std::endl;
                        d_i_j(i) = ROBOT_RANGE;
                        theta_left(i) = 2*M_PI;
                    }
                }
            }

            // Get distances
            for (int i = 0; i < NEIGHBORS_NUM; i++)
            {
                if (p_j(0,i) != 100.0)
                {
                    d_i_j(i) = sqrt(pow(p_i(0)-p_j(0,i),2) + pow(p_i(1)-p_j(1,i),2));
                }
                else
                {
                    d_i_j(i) -= wd;                     // worst case: distance decreases as neighbor is moving towards me
                    std::cout << "Robot " << i << " is lost." << std::endl;
                    std::cout << "Worst case distance: " << d_i_j(i) << std::endl;
                }
            }

            std::cout << "Distances: " << d_i_j.transpose() << std::endl;

            // Get weighted average position of neighbors
            Eigen::Vector3d p_j_avg;                        // (x,y,theta) with theta = 0.0 always
            p_j_avg << 0.0,0.0,0.0;
            double sum = 0.0;
            for (int i = 0; i < NEIGHBORS_NUM; i++)
            {
                if (p_j(0,i) != 100.0 && d_i_j(i) > 0.0)
                {
                    p_j_avg(0) += 1/d_i_j(i)*p_j(0,i);          // weight = 1/distance
                    p_j_avg(1) += 1/d_i_j(i)*p_j(1,i);
                    sum += 1/d_i_j(i);
                }
            }
            if (sum != 0.0)                                 // if no neighbors detected: sum = 0 --> avg = NaN
            {
                p_j_avg(0) = p_j_avg(0)/sum;
                p_j_avg(1) = p_j_avg(1)/sum;
            }
            std::cout << "Weighted average position of neighbors: " << p_j_avg(0) << " " << p_j_avg(1) << std::endl;

            // DEBUG: publish avg point
            geometry_msgs::PoseStamped avg_point;
            avg_point.header.stamp = ros::Time::now();
            avg_point.header.frame_id = "hummingbird1/base_link";
            avg_point.pose.position.x = p_j_avg(0);
            avg_point.pose.position.y = p_j_avg(1);
            avg_point.pose.position.z = 0.0;
            avg_pub.publish(avg_point);

            // Get control action to face p_j_avg
            if (p_j_avg(0) != 0.0 && p_j_avg(1) != 0.0)                     // if 0.0 --> no neighbors detected
            {
                double theta_des = atan2(p_j_avg(1),p_j_avg(0));
                if (controller.calculate(theta_des, 0.0) == controller.calculate(theta_des, 0.0))       // check for NaN
                {
                    w += controller.calculate(theta_des, 0.0);
                    std::cout << "Orientation correction action: " << controller.calculate(theta_des, 0.0) << std::endl;
                }
            }
            std::cout << "Overall corrective action: " << w << std::endl;
            // w = 0.5*theta_des;
            // std::cout << "Calculated control action: " << w << std::endl;

            // Check if w = nan
            // if (w != w) w = 0.0;

            // Actuate robot
            geometry_msgs::TwistStamped vel_msg;
            vel_msg.twist.linear.x = u_des(0);
            vel_msg.twist.linear.y = u_des(1);
            vel_msg.twist.angular.z = u_des(2) + w;
            pub.publish(vel_msg);
        }

        void neigh_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
        {
            int c = 0;
            for (int i=0; i<msg->poses.size(); i++)
            {
                p_j(0,i) = msg->poses[i].position.x;
                p_j(1,i) = msg->poses[i].position.y;
                p_j(2,i) = tf2::getYaw(msg->poses[i].orientation);

                if (msg->poses[i].position.x != 100.0)
                {
                    c++;
                }
            }

            neighbors_count = c;
            std::cout << "Number of neighbors: " << neighbors_count << std::endl;
        }
        
        void pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
            {
                p_i(0) = msg->pose.pose.position.x;
                p_i(1) = msg->pose.pose.position.y;
                p_i(2) = tf2::getYaw(msg->pose.pose.orientation);
            }

        void vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
        {
            u_des(0) = msg->linear.x;
            u_des(1) = msg->linear.y;
            u_des(2) = msg->angular.z;
        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");
    HeadingController HC;

    ros::spin();

    return 0;
}