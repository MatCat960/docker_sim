#include <pid_control/PIDController.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

class Subscribe_And_Publish
{
    private:
        ros::Publisher pub;
        ros::Subscriber vel_sub;
        ros::Subscriber pose_sub;
        ros::Subscriber neighbor_sub;
        ros::NodeHandle n;
        ros::Timer timer;
        PID::PIDController controller;
        Eigen::Vector3d p_i;
        Eigen::MatrixXd p_j;

    public:
        Subscribe_And_Publish(): controller(0.1, 100, -100, 0.5, 0.01, 0.5)
        {
            vel_sub = n.subscribe<geometry_msgs::Twist> ("/cmd_vel_in", 1, &Subscribe_And_Publish::vel_callback, this);
            neighbor_sub = n.subscribe<geometry_msgs::PoseArray>("/supervisor/robot0/pose",1,&Subscribe_And_Publish::neigh_callback,this);
            pose_sub = n.subscribe<nav_msgs::Odometry>("/hummingbird1/ground_truth/odometry",1,&Subscribe_And_Publish::pose_callback,this);
            pub = n.advertise<geometry_msgs::TwistStamped> ("/hummingbird1/autopilot/velocity_command", 1);
            timer = n.createTimer(ros::Duration(0.1), &Subscribe_And_Publish::timerCallback,this);
            
            p_i << 0.0,0.0,0.0;
            p_j.resize(3,2);
            p_j << 100.0,0.0,0.0,100.0,0.0,0.0;

            std::cout << "Initialized." << std::endl;
        
        }   

        void timerCallback(const ros::TimerEvent&)
        {
            geometry_msgs::TwistStamped vel_msg;
            std::cout << "Current theta: " << p_i(2) << std::endl;
            double theta = controller.calculate(0.5*M_PI, p_i(2));
            std::cout << "Calculated control action: " << theta << std::endl;

            vel_msg.twist.angular.z = theta;
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
        }
        
        void pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
            {
                p_i(0) = msg->pose.pose.position.x;
                p_i(1) = msg->pose.pose.position.y;
                p_i(2) = tf2::getYaw(msg->pose.pose.orientation);
            }

        void vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
        {
            std::cout << "CIAO" << std::endl;
        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_controller");
    Subscribe_And_Publish SAPObject;

    ros::spin();

    return 0;
}