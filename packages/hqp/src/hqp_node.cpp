#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

#include <hqp/Hqp.h>


class Subscribe_And_Publish
{
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle n;
    ros::Timer timer;
    Hqp hqp_solver;
    Eigen::VectorXd delta;
    Eigen::MatrixXd p_j, slacks;

public:
    Subscribe_And_Publish(): hqp_solver(1.57,1.0,6.0,3)
    {
        // sub = n.subscribe<geometry_msgs::Twist> ("/cmd_vel_in", 1, &Subscribe_And_Publish::vel_callback, this);
        // pose_subs.push_back(n.subscribe<geometry_msgs::Pose>("/pose1",1,[this](const geometry_msgs::Pose::ConstPtr &msg){this->pose_callback(msg,1);}));
        // pose_subs.push_back(n.subscribe<geometry_msgs::Pose>("/pose2",1,[this](const geometry_msgs::Pose::ConstPtr &msg){this->pose_callback(msg,2);}));
        // pose_subs.push_back(n.subscribe<geometry_msgs::Pose>("/pose3",1,[this](const geometry_msgs::Pose::ConstPtr &msg){this->pose_callback(msg,3);}));
        // pub = n.advertise<geometry_msgs::TwistStamped> ("/cmd_vel_out", 1);
        timer = n.createTimer(ros::Duration(2.0), &Subscribe_And_Publish::timerCallback,this);
        hqp_solver.setVerbose(false);
        
        // p_i << 0.0,0.0,0.0;
        p_j.resize(3,3);
        p_j << 2.5, 1.0, 15.0,
                1.0, 4.2, 11.0,
                0.0, 0.0, 0.0;

        slacks.resize(4*p_j.cols(), 4);
        slacks.setZero();

        // delta.resize(3); 
    }

    void timerCallback(const ros::TimerEvent&)
    {
        // if (!hqp_solver.solve(p_j))
        // {
        //     ROS_INFO("HQP Solved");
        // }
        // else
        // {
        //     ROS_INFO("HQP Not solved");
        // }
        hqp_solver.solve(p_j);
    }

    

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    ros::init(argc, argv, "front_end");
    Subscribe_And_Publish SAPObject;

    ros::spin();

    return 0;
}