// STL
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cstdlib>
#include <cmath>
#include <netinet/in.h>
#include <sys/types.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
// My includes
#include "FortuneAlgorithm.h"
#include "Voronoi.h"
// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>
// #include "coverage_unimore_nyu/PoseVector.h"

//------------------------------------------------------------------------
const int debug = 0;
//------------------------------------------------------------------------
const int shutdown_timer = 5;           //count how many seconds to let the robots stopped before shutting down the node
//------------------------------------------------------------------------
//------------------------------------------------------------------------
sig_atomic_t volatile node_shutdown_request = 0;    //signal manually generated when ctrl+c is pressed
//------------------------------------------------------------------------

bool IsPathExist(const std::string &s)
{
  struct stat buffer;
  return (stat (s.c_str(), &buffer) == 0);
}

class Controller
{
public:
    Controller() : nh_priv_("~")
    {
        ROS_INFO("Node Initialization");
        //-------------------------------------------------------- ROS parameters -----------------------------------------------------------
        this->nh_priv_.getParam("ids", ids_);
        this->nh_priv_.getParam("ROBOTS_NUM", ROBOTS_NUM);

        //sensing range single robot (= halfe edge of the local sensing box)
        this->nh_priv_.getParam("ROBOT_RANGE", ROBOT_RANGE);
        this->nh_priv_.getParam("ROBOT_FOV", ROBOT_FOV);

        // Parameters for Gaussian

        // Area parameter
        this->nh_priv_.getParam("AREA_SIZE_x", AREA_SIZE_x);
        this->nh_priv_.getParam("AREA_SIZE_y", AREA_SIZE_y);
        this->nh_priv_.getParam("AREA_LEFT", AREA_LEFT);
        this->nh_priv_.getParam("AREA_BOTTOM", AREA_BOTTOM);

        //view graphical voronoi rapresentation - bool
        //-----------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------- Subscribers and Publishers ----------------------------------------------------
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            // TODONYU: change the topic name to match the one you use
            poseSub_.push_back(nh_.subscribe<nav_msgs::Odometry>("/turtlebot" + std::to_string(i) + "/odom", 1, std::bind(&Controller::poseCallback, this, i, std::placeholders::_1)));
            neighposePub_.push_back(nh_.advertise<geometry_msgs::PoseArray>("/supervisor/robot" + std::to_string(i) + "/pose", 1));
        }
        controller_timer_ = nh_.createTimer(ros::Duration(0.1), std::bind(&Controller::Emulate_Vision, this));
        //-----------------------------------------------------------------------------------------------------------------------------------
        //----------------------------------------------------------- init Variables ---------------------------------------------------------
        pose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
        //------------------------------------------------------------------------------------------------------------------------------------

        //----------------------------------------------- Graphics window -----------------------------------------------
        //---------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------open log file -----------------------------------------------
        //---------------------------------------------------------------------------------------------------------------
        ROBOT_FOV_rad = 0.5 * ROBOT_FOV /180 * M_PI;
    }

    ~Controller()
    {
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
        //ros::shutdown();
    }

    void stop();
    void poseCallback(int i, const nav_msgs::Odometry::ConstPtr &msg);
    void Emulate_Vision();

    //Graphics -- draw coverage

    //open write and close LOG file

private:
    std::vector<int> ids_{1,2,3};
    int ROBOTS_NUM = 3;
    double ROBOT_RANGE = 5;
    double ROBOT_FOV = 100.0;
    double ROBOT_FOV_rad;
    double vel_linear_x, vel_angular_z;
    Eigen::VectorXd pose_x;
    Eigen::VectorXd pose_y;
    Eigen::VectorXd pose_theta;
    std::vector<Vector2<double>> seeds_xy;

    

    //------------------------------- Ros NodeHandle ------------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    //-----------------------------------------------------------------------------------
    //------------------------- Publishers and subscribers ------------------------------
    std::vector<ros::Publisher> neighposePub_;
    std::vector<ros::Subscriber> poseSub_;
    ros::Timer controller_timer_;
    //-----------------------------------------------------------------------------------

    //Rendering with SFML
    //------------------------------ graphics window -------------------------------------
    //------------------------------------------------------------------------------------

    //---------------------------- Environment definition --------------------------------
    double AREA_SIZE_x = 20.0;
    double AREA_SIZE_y = 20.0;
    double AREA_LEFT = -10.0;
    double AREA_BOTTOM = -10.0;
    //------------------------------------------------------------------------------------

    //---------------------- Gaussian Density Function parameters ------------------------
    //------------------------------------------------------------------------------------

    //graphical view - ON/OFF

    //timer - check how long robots are being stopped

    //ofstream on external log file
};

void Controller::stop()
{
    ROS_INFO("shutting down the supervisor controller");
    ros::Duration(0.1).sleep();

    ros::shutdown();
}

void Controller::poseCallback(int i, const nav_msgs::Odometry::ConstPtr &msg)
{
    this->pose_x(i) = msg->pose.pose.position.x;
    this->pose_y(i) = msg->pose.pose.position.y;  

    this->pose_theta(i) = tf2::getYaw(msg->pose.pose.orientation);;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------Rendering functions - for SFML graphical visualization-----------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Controller::Emulate_Vision(){

    Box<double> AreaBox{AREA_LEFT, AREA_BOTTOM, AREA_SIZE_x + AREA_LEFT, AREA_SIZE_y + AREA_BOTTOM};
    Box<double> RangeBox{-ROBOT_RANGE, -ROBOT_RANGE, ROBOT_RANGE, ROBOT_RANGE};

    for (int i = 0; i < ROBOTS_NUM; ++i)
    {
        geometry_msgs::PoseArray neighbors;
        
        neighbors.header.stamp = ros::Time::now();
        std::stringstream ss;
        ss << "hummingbird"<< i+1<<"/base_link";
        neighbors.header.frame_id = ss.str();

        for (int j = 0; j < ROBOTS_NUM; ++j)
        {
            if (i != j)
            {
                Vector2<double> distance_vect = {(this->pose_x[j] - this->pose_x[i]), (this->pose_y[j] - this->pose_y[i])};
                
                if (distance_vect.getNorm() <= ROBOT_RANGE)
                {
                    geometry_msgs::Pose msg;
                    //the distance_vect is already the point neighbor expressed in local coordinates
                    msg.position.x = cos(this->pose_theta(i))*distance_vect.x + sin(this->pose_theta(i))*distance_vect.y;
                    msg.position.y = -sin(this->pose_theta(i))*distance_vect.x + cos(this->pose_theta(i))*distance_vect.y;
                    msg.orientation.w = 1.0;

                    // Filter robots that are not in the FOV
                    double angle = abs(atan2(msg.position.y, msg.position.x));
                    if (angle <= ROBOT_FOV_rad &&  msg.position.x > 0.0)
                        neighbors.poses.push_back(msg);
                }
            }
        }

        std::cout<<"robot "<<i<<" has "<<neighbors.poses.size()<<" neighbours"<<std::endl;
        this->neighposePub_[i].publish(neighbors);
    }
}

/*******************************************************************************
* Main function
*******************************************************************************/
//alternatively to a global variable to have access to the method you can make STATIC the class method interested, 
//but some class function may not be accessed: "this->" method cannot be used

void nodeobj_wrapper_function(int){
    ROS_WARN("signal handler function CALLED");
    node_shutdown_request = 1;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_coverage_supervisor", ros::init_options::NoSigintHandler);
    signal(SIGINT, nodeobj_wrapper_function);

    //Controller node_controller;
    auto node_controller = std::make_shared<Controller>();

    while (!node_shutdown_request){
        ros::spinOnce();
    }
    node_controller->stop();

    //ros::spin();
    //do pre-shutdown tasks
    if (ros::ok())
    {
        ROS_WARN("ROS HAS NOT BEEN PROPERLY SHUTDOWN, it is being shutdown again.");
        ros::shutdown();
    }

    return 0;
}