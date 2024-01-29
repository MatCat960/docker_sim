// STL
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <netinet/in.h>
#include <sys/types.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <math.h>
#include <signal.h>
// SFML
// #include <SFML/Graphics.hpp>
// #include <SFML/OpenGL.hpp>
// My includes
#include "Graphics.h"
#include <SFML/Window/Mouse.hpp>
// ROS includes
#include <ros/ros.h>
// #include "std_msgs/msg/string.hpp"
// #include "geometry_msgs/msg/point.hpp"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>


#define M_PI   3.14159265358979323846  /*pi*/

using namespace std::chrono_literals;
using std::placeholders::_1;

sig_atomic_t volatile node_shutdown_request = 0;    //signal manually generated when ctrl+c is pressed

std::vector<std::string> readFile(std::string path){
    // Create and open a stream to read the file contents by giving the path to the file
    std::ifstream file(path);
    std::vector<std::string> lines;
    std::string line;
    
    if(file.is_open()) {
        // Read the file line by line
        while(getline(file, line)) {
            lines.push_back(line);
        }
        
        file.close();
    }else {
        std::cerr << "Unable to open file: " << path << std::endl;
    }

    return lines;
}

std::vector<double> getNumbers(const std::string &line)
{
    // get numbers even if I dont know the number of digits
    std::vector<double> numbers;
    int start = 0;
    int length = 5;

    std::string n0 = line.substr(start, length);
    std::string n1 = line.substr(start + length + 1, length);
    std::string n2 = line.substr(start + 2*length + 2, length);
    numbers.push_back(std::stod(n0));
    numbers.push_back(std::stod(n1));
    numbers.push_back(std::stod(n2));

    return numbers;
}

class Controller
{

public:
    Controller() : nh_priv_("~")
    {
        //------------------------------------------------- ROS parameters ---------------------------------------------------------
        // Number of robots
        this->nh_priv_.getParam("ROBOTS_NUM", ROBOTS_NUM);

        // Name of the controlled robot
        this->nh_priv_.getParam("NAME", name);

        // File containing positions
        this->nh_priv_.getParam("FILE", filepath);

        

        //--------------------------------------------------- Subscribers and Publishers ----------------------------------------------------    
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            odomPubs_.push_back(nh_.advertise<nav_msgs::Odometry>("/robot" + std::to_string(i) + "/odom", 1));
            velSubs_.push_back(nh_.subscribe<geometry_msgs::TwistStamped>("/robot" + std::to_string(i) + "/cmd_vel", 1, std::bind(&Controller::velCallback, this, std::placeholders::_1, i)));
        }

        std::cout << "ROS Subscribers and Publishers have been initialized" << std::endl;

        timer_ = nh_.createTimer(ros::Duration(0.25), std::bind(&Controller::loop, this));

        // Get starting position from file
        std::vector<std::string> lines = readFile(filepath);
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            std::vector<double> numbers = getNumbers(lines[i]);
            px.push_back(numbers[0]);
            py.push_back(numbers[1]);
            pth.push_back(numbers[2]);

            // Initialize velocities
            vx.push_back(0.0);
            vy.push_back(0.0);
            vth.push_back(0.0);

            // Initialize odom msg
            nav_msgs::Odometry odom_msg;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = name+std::to_string(i);
            odom_msgs.push_back(odom_msg);
        }

        std::cout << "Robots positions and velocities have been initialized" << std::endl;


    }
    ~Controller()
    {
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
    }

    void stop();
    void velCallback(const geometry_msgs::TwistStamped::ConstPtr msg, int i);
    void loop();


private:
    int ROBOTS_NUM = 6;
    std::string name = "uav";
    std::vector<double> vx;
    std::vector<double> vy;
    std::vector<double> vth;

    std::vector<double> px;
    std::vector<double> py;
    std::vector<double> pth;

    double VMAX = 0.5;

    double dt = 0.1;

    std::string filepath = "path.txt";

    double timerstart = 0.0;

    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    std::vector<ros::Subscriber> velSubs_;
    std::vector<ros::Publisher> odomPubs_;
    ros::Timer timer_;

    std::vector<nav_msgs::Odometry> odom_msgs;

};


void Controller::stop()
{
    //if (signum == SIGINT || signum == SIGKILL || signum ==  SIGQUIT || signum == SIGTERM)
    ROS_INFO("shutting down the controller, stopping the robots, closing the graphics window");

    ROS_INFO("controller has been closed and robots have been stopped");
    ros::Duration(0.1).sleep();

    ros::shutdown();
}

void Controller::velCallback(const geometry_msgs::TwistStamped::ConstPtr msg, int i)
{
    vx[i] = std::max(-VMAX, std::min(msg->twist.linear.x, VMAX));
    vy[i] = std::max(-VMAX, std::min(msg->twist.linear.y, VMAX));
    vth[i] = std::max(-VMAX, std::min(msg->twist.angular.z, VMAX));
}

void Controller::loop()
{
    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        px[i] += vx[i] * dt;
        py[i] += vy[i] * dt;
        pth[i] += vth[i] * dt;

        // Publish odometry
        odom_msgs[i].header.stamp = ros::Time::now();
        odom_msgs[i].pose.pose.position.x = px[i];
        odom_msgs[i].pose.pose.position.y = py[i];
        odom_msgs[i].pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), pth[i]));

        odomPubs_[i].publish(odom_msgs[i]);
    }
}


//alternatively to a global variable to have access to the method you can make STATIC the class method interested, 
//but some class function may not be accessed: "this->" method cannot be used

std::shared_ptr<Controller> globalobj_signal_handler;     //the signal function requires only one argument {int}, so the class and its methods has to be global to be used inside the signal function.
void nodeobj_wrapper_function(int){
    std::cout<<"signal handler function CALLED"<<std::endl;
    node_shutdown_request = 1;
}

int main(int argc, char **argv)
{
    // signal(SIGINT, nodeobj_wrapper_function);

    ros::init(argc, argv, "centralized_drone_sim");
    auto node = std::make_shared<Controller>();

    // globalobj_signal_handler = node;    //to use the ros function publisher, ecc the global pointer has to point to the same node object.


    // rclcpp::spin(node);

    // rclcpp::sleep_for(100000000ns);
    // rclcpp::shutdown();

    
    while(!node_shutdown_request)
    {
        ros::spinOnce();
    }
    node->stop();

    if (ros::ok())
    {
        ROS_WARN("ROS HAS NOT BEEN PROPERLY SHUTDOWN, it is being shutdown again.");
        ros::shutdown();
    }
    

    return 0;
}
