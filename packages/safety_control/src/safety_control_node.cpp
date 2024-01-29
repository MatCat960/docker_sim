#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <safety_control/SafetyController.h>
#include <tf2/utils.h>

class Subscribe_And_Publish
{
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber neigh_sub;
    std::vector<ros::Subscriber> pose_subs;
    ros::NodeHandle n;
    ros::Timer timer;
    safety_control::SafetyController controller;
    Eigen::Vector3d ustar,p_i;
    Eigen::VectorXd h;
    Eigen::MatrixXd p_j;
    int detected_robots;

public:


    Subscribe_And_Publish(): controller(1.0, 6.0, 2)
    {
        std::cout << "Starting initialization..." << std::endl;
        sub = n.subscribe<geometry_msgs::Twist> ("/cmd_vel", 1, &Subscribe_And_Publish::vel_callback, this);
        pose_subs.push_back(n.subscribe<geometry_msgs::Pose>("/pose1",1,[this](const geometry_msgs::Pose::ConstPtr &msg){this->pose_callback(msg,1);}));
        pose_subs.push_back(n.subscribe<geometry_msgs::Pose>("/pose2",1,[this](const geometry_msgs::Pose::ConstPtr &msg){this->pose_callback(msg,2);}));
        pose_subs.push_back(n.subscribe<geometry_msgs::Pose>("/pose3",1,[this](const geometry_msgs::Pose::ConstPtr &msg){this->pose_callback(msg,3);}));
        // neigh_sub = n.subscribe<geometry_msgs::PoseArray>("/supervisor/robot0/pose", 1, &Subscribe_And_Publish::neigh_callback, this);
        pub = n.advertise<geometry_msgs::TwistStamped> ("/cmd_vel_out", 1);
        timer = n.createTimer(ros::Duration(0.1), &Subscribe_And_Publish::timerCallback,this);
        
        std::cout << "ROS STUFF INITIALIZED." << std::endl;
        p_i << 0.0,0.0,0.0;
        p_j.resize(3,2);
                
        p_j << 100.0,0.0,0.0,100.0,0.0,0.0;

        std::cout << "p_i and p_js INITIALIZED." << std::endl;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        Eigen::Vector3d uopt;
        geometry_msgs::TwistStamped vel_msg;
        // vel_msg.header.frame_id = "hummingbird1/base_link";
        std::vector<double> vec = {1.0, 1.0};
        if(!controller.applyCbfLocal(uopt,h,ustar,p_j, vec))
        {        
            vel_msg.twist.linear.x = uopt(0);
            vel_msg.twist.linear.y = uopt(1);
            vel_msg.twist.angular.z = uopt(2);
            if(vel_msg.twist.linear.x > 1.0)
            {
                vel_msg.twist.linear.x = 1.0;
            } else if (vel_msg.twist.linear.x < -1.0)
            {
                vel_msg.twist.linear.x = -1.0;
            }
            if(vel_msg.twist.linear.y > 1.0)
            {
                vel_msg.twist.linear.y = 1.0;
            } else if (vel_msg.twist.linear.y < -1.0)
            {
                vel_msg.twist.linear.y = -1.0;
            }
            if(vel_msg.twist.angular.z > 1.0)
            {
                vel_msg.twist.angular.z = 1.0;
            } else if (vel_msg.twist.angular.z < -1.0)
            {
                vel_msg.twist.angular.z = -1.0;
            }
            pub.publish(vel_msg);
            ROS_INFO("CBF SUCCESS");
        }
        else{
            ROS_INFO("CBF FAILED");
            ROS_ERROR("CBF FAILED");
        }
    }

    // Ricevo posizione globale di tutti i robot! Conversione in coordinate locali avviene all'interno del codice
    void pose_callback(const geometry_msgs::Pose::ConstPtr &msg,int i){
        if (i == 1){
            //ROS_INFO("GOT POSE 1");
            p_i(0) = msg->position.x;
            p_i(1) = msg->position.y;
            p_i(2) = tf2::getYaw(msg->orientation);
        }
        else{
            //ROS_INFO("GOT POSE %d",i);
            // p_j(0,i-2) = msg->position.x - p_i(0);
            // p_j(1,i-2) = msg->position.y - p_i(1);
            // p_j(2,i-2) = tf2::getYaw(msg->orientation);

            double dist_x = msg->position.x - p_i(0);
            double dist_y = msg->position.y - p_i(1);

            p_j(0,i-2) = dist_x * cos(p_i(2)) + dist_y * sin(p_i(2));
            p_j(1,i-2) = -dist_x * sin(p_i(2)) + dist_y * cos(p_i(2));
            p_j(2,i-2) = tf2::getYaw(msg->orientation);
        }
    }

    // void neigh_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
    // {
    //     detected_robots = msg->poses.size();
    //     for (int i = 0; i < detected_robots; i++)
    //     {
    //         p_j(0,i) = msg->poses[i].position.x;
    //         p_j(1,i) = msg->poses[i].position.y;
    //         p_j(2,i) = tf2::getYaw(msg->poses[i].orientation);
    //     }
    //     for (int i=detected_robots; i<2; i++)
    //     {
    //         p_j(0,i) = 100.0;
    //         p_j(1,i) = 0.0;
    //         p_j(2,i) = 0.0;
    //     }
    //     std::cout << "Number of neighbors: " << detected_robots << std::endl;
    // }

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