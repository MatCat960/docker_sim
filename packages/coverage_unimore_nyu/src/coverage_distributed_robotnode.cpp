// #define ARPL_
// STL
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
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
#include "fow_control/FowController.h"
// ROS includes
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>
#include "coverage_unimore_nyu/PoseVector.h"

// ARPL UAV support
#ifdef ARPL_
#include <mav_manager/manager.h>
#include <mav_manager/Vec4.h>
#endif

// Robots parameters ------------------------------------------------------
const double MAX_ANG_VEL = 0.3;
const double MAX_LIN_VEL = 1.0; // set to turtlebot max velocities
const double b = 0.025;         // for differential drive control (only if we are moving a differential drive robot (e.g. turtlebot))
//------------------------------------------------------------------------
const float CONVERGENCE_TOLERANCE = 0.05;
//------------------------------------------------------------------------
const int debug = 0;
//------------------------------------------------------------------------
const int shutdown_timer = 5; // count how many seconds to let the robots stopped before shutting down the node
//------------------------------------------------------------------------
//------------------------------------------------------------------------
sig_atomic_t volatile node_shutdown_request = 0; // signal manually generated when ctrl+c is pressed
//------------------------------------------------------------------------

class Controller
{
public:
    Controller() : nh_priv_("~"), fow_controller(2.618, 0.5, 10.0, 5)//hardcoded field of view, safe distance, max distance and max nr of robots
    
    {
        ROS_INFO("Node Initialization");
        //-------------------------------------------------------- ROS parameters -----------------------------------------------------------
        this->nh_priv_.getParam("ROBOTS_NUM", ROBOTS_NUM);
        this->nh_priv_.getParam("MODEL_NAME", MODEL_NAME);

        // sensing range single robot (= halfe edge of the local sensing box)
        this->nh_priv_.getParam("ROBOT_RANGE", ROBOT_RANGE);

        // Parameters for Gaussian
        this->nh_priv_.getParam("GAUSSIAN_DISTRIBUTION", GAUSSIAN_DISTRIBUTION);
        this->nh_priv_.getParam("PT_X", PT_X);
        this->nh_priv_.getParam("PT_Y", PT_Y);
        this->nh_priv_.getParam("VAR", VAR);

        // Area parameter
        this->nh_priv_.getParam("AREA_SIZE_x", AREA_SIZE_x);
        this->nh_priv_.getParam("AREA_SIZE_y", AREA_SIZE_y);
        this->nh_priv_.getParam("AREA_LEFT", AREA_LEFT);
        this->nh_priv_.getParam("AREA_BOTTOM", AREA_BOTTOM);
        use_cbf_ = this->nh_priv_.param("use_cbf",true);

        // view graphical voronoi rapresentation - bool

        // Robot ID
        this->nh_priv_.getParam("ROBOT_ID", ROBOT_ID);

        //-----------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------- Subscribers and Publishers ----------------------------------------------------
        // TODONYU: change the topic name to match the one you use
        velPub_ = nh_.advertise<geometry_msgs::TwistStamped>("/turtlebot" + std::to_string(ROBOT_ID) + "/cmd_vel", 1);
        // neighposeSub_ = nh_.subscribe<geometry_msgs::PoseArray>("/supervisor/robot" + std::to_string(ROBOT_ID) + "/pose", 1, std::bind(&Controller::neighposeCallback, this, std::placeholders::_1));
        std::string polyTemplate = "/turtlebot"+ std::to_string(ROBOT_ID) + "/voronoi_cell";
        std::string centroidTemplate = "/turtlebot"+ std::to_string(ROBOT_ID) + "/voronoi_centroid";

        // poses subscribers
        for (int i = 0; i < ROBOTS_NUM; ++i)
        {
            std::string topic = "/turtlebot" + std::to_string(i) + "/odom";
            neighposeSub_.push_back(nh_.subscribe<nav_msgs::Odometry>(topic, 1, std::bind(&Controller::odomCallback, this, std::placeholders::_1, i)));
        }
  
        // voronoi cell publishers
        polyPub_ = nh_.advertise<geometry_msgs::PolygonStamped>(polyTemplate,1);
        // voronoi centroid publishers
        centrPub_ = nh_.advertise<visualization_msgs::Marker>(centroidTemplate,1);
        controller_timer_ = nh_.createTimer(ros::Duration(0.1), std::bind(&Controller::Coverage, this));
        // publish cbf h function
        
        std::string cbfTemplate = "/turtlebot"+ std::to_string(ROBOT_ID) + "/cbf_h";
        hfuncPub_ = nh_.advertise<std_msgs::Float64MultiArray>(cbfTemplate,1);
        //-----------------------------------------------------------------------------------------------------------------------------------
        //----------------------------------------------------------- init Variables ---------------------------------------------------------
        pose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_z = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
        time(&this->timer_init_count);
        time(&this->timer_final_count);
//------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------ARPL ROS SERVICE -----------------------------------------------
#ifdef ARPL_
        sc_setDesVelInWorldFrame = nh_.serviceClient<mav_manager::Vec4>("/" + MODEL_NAME + "/mav_services/setDesVelInWorldFrame");
#endif
        //---------------------------------------------------------------------------------------------------------------
        //----------------------------------------------- Graphics window -----------------------------------------------

        //---------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------open log file -----------------------------------------------
        //---------------------------------------------------------------------------------------------------------------
    }

    ~Controller()
    {
        std::cout << "DESTROYER HAS BEEN CALLED" << std::endl;
        // ros::shutdown();
    }

    void stop();
    // void neighposeCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, int i);
    void Coverage();
    geometry_msgs::Twist Diff_drive_compute_vel(double vel_x, double vel_y, double alfa);
    std::vector<Vector2<double>> erase_1st(std::vector<Vector2<double>> input_vect);

    // Graphics -- draw coverage

private:
    int ROBOTS_NUM = 6;
    int ROBOT_ID = 0;
    double ROBOT_RANGE = 5;
    std::string MODEL_NAME;
    double vel_linear_x, vel_angular_z;
    Eigen::VectorXd pose_x;
    Eigen::VectorXd pose_y;
    Eigen::VectorXd pose_z;
    Eigen::VectorXd pose_theta;
    bool use_cbf_=false;

    std::vector<Vector2<double>> seeds_xy;
    fow_control::FowController fow_controller;
    int neighbours_num;
    //------------------------------- Ros NodeHandle ------------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    //-----------------------------------------------------------------------------------
    //------------------------- Publishers and subscribers ------------------------------
    ros::Publisher velPub_, polyPub_, centrPub_,hfuncPub_;
    std::vector<ros::Subscriber> neighposeSub_;
    ros::Timer controller_timer_;
    //-----------------------------------------------------------------------------------

    // Rendering with SFML
    //------------------------------ graphics window -------------------------------------

    //------------------------------------------------------------------------------------

    //---------------------------- Environment definition --------------------------------
    double AREA_SIZE_x = 20.0;
    double AREA_SIZE_y = 20.0;
    double AREA_LEFT = -10.0;
    double AREA_BOTTOM = -10.0;
    //------------------------------------------------------------------------------------

    //---------------------- Gaussian Density Function parameters ------------------------
    bool GAUSSIAN_DISTRIBUTION = false;
    double PT_X = 8.0;
    double PT_Y = 8.0;
    double VAR = 0.4;
    //------------------------------------------------------------------------------------

    // graphical view - ON/OFF

    // timer - check how long robots are being stopped
    time_t timer_init_count;
    time_t timer_final_count;
#ifdef ARPL_
    ros::ServiceClient sc_setDesVelInWorldFrame;
#endif
};

void Controller::stop()
{
    // if (signum == SIGINT || signum == SIGKILL || signum ==  SIGQUIT || signum == SIGTERM)
    ROS_INFO("shutting down the controller, stopping the robots, closing the graphics window");

    ros::Duration(0.1).sleep();

    geometry_msgs::TwistStamped vel_msg;
    for (int r = 0; r < ROBOTS_NUM; ++r)
    {
        this->velPub_.publish(vel_msg);
    }

    ROS_INFO("controller has been closed and robots have been stopped");
    ros::Duration(0.1).sleep();

    ros::shutdown();
}

// void Controller::neighposeCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
// {
//     pose_x = 100*Eigen::VectorXd::Ones(ROBOTS_NUM);
//     pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
//     pose_z = Eigen::VectorXd::Zero(ROBOTS_NUM);
//     // pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
//     for (unsigned int i = 0; i < msg->poses.size(); ++i)
//     {
//         auto x = msg->poses[i].position.x;
//         auto y = msg->poses[i].position.y;
//         auto z = msg->poses[i].position.z;
//         auto theta = tf2::getYaw(msg->poses[i].orientation);

//         std::cout << "robot -" << i << ": " << x << ", " << y << ", " << theta << std::endl;

//         this->pose_x(i) = msg->poses[i].position.x;
//         this->pose_y(i) = msg->poses[i].position.y;
//         this->pose_z(i) = msg->poses[i].position.z;
//         this->pose_theta(i) = tf2::getYaw(msg->poses[i].orientation);
//     }
//     neighbours_num = msg->poses.size()-1;
// }

void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr &msg, int i)
{
    auto x = msg->pose.pose.position.x;
    auto y = msg->pose.pose.position.y;
    // auto z = msg->pose.pose.position.z;
    auto theta = tf2::getYaw(msg->pose.pose.orientation);

    // std::cout << "robot -" << i << ": " << x << ", " << y << ", " << theta << std::endl;

    this->pose_x(i) = msg->pose.pose.position.x;
    this->pose_y(i) = msg->pose.pose.position.y;
    // this->pose_z(i) = msg->pose.pose.position.z;
    this->pose_theta(i) = tf2::getYaw(msg->pose.pose.orientation);

}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------Rendering functions - for SFML graphical visualization-----------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
geometry_msgs::Twist Controller::Diff_drive_compute_vel(double vel_x, double vel_y, double alfa)
{
    //-------------------------------------------------------------------------------------------------------
    // Compute velocities commands for the robot: differential drive control, for UAVs this is not necessary
    //-------------------------------------------------------------------------------------------------------

    geometry_msgs::Twist vel_msg;
    // double alfa = (this->pose_theta(i));
    double v = 0, w = 0;

    v = cos(alfa) * vel_x + sin(alfa) * vel_y;
    w = -(1 / b) * sin(alfa) * vel_x + (1 / b) * cos(alfa) * vel_y;

    if (abs(v) <= MAX_LIN_VEL)
    {
        vel_msg.linear.x = v;
    }
    else
    {
        if (v >= 0)
        {
            vel_msg.linear.x = MAX_LIN_VEL;
        }
        else
        {
            vel_msg.linear.x = -MAX_LIN_VEL;
        }
    }

    if (abs(w) <= MAX_ANG_VEL)
    {
        vel_msg.angular.z = w;
    }
    else
    {
        if (w >= 0)
        {
            vel_msg.angular.z = MAX_ANG_VEL;
        }
        else
        {
            vel_msg.angular.z = -MAX_ANG_VEL;
        }
    }
    return vel_msg;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
std::vector<Vector2<double>> Controller::erase_1st(std::vector<Vector2<double>> input_vect)
{
    std::vector<Vector2<double>> output_vect;
    for (size_t i = 1; i < input_vect.size(); ++i)
    {
        output_vect.push_back(input_vect[i]);
    }
    return output_vect;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Controller::Coverage()
{
    // Parameters
    // double min_dist = 0.4;         //avoid robot collision
    int K_gain = 1; // Lloyd law gain

    // Variables
    double vel_x = 0, vel_y = 0;
    std::vector<Vector2<double>> seeds;
    std::vector<Vector2<double>> centroids;
    Vector2<double> vel;
    Vector2<double> centroid;

    Box<double> AreaBox{AREA_LEFT, AREA_BOTTOM, AREA_SIZE_x + AREA_LEFT, AREA_SIZE_y + AREA_BOTTOM};
    Box<double> RangeBox{-ROBOT_RANGE, -ROBOT_RANGE, ROBOT_RANGE, ROBOT_RANGE};

    seeds[0] = {this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID)};

    for (int i = 0; i < ROBOTS_NUM; ++i)
    {
        if ((i != ROBOT_ID) && (this->pose_x[i] != 0.0) && (this->pose_y[i] != 0.0) && (this->pose_x[i] != 100.0) && (this->pose_y[i] != 100.0))
        {
            seeds.push_back({this->pose_x[i], this->pose_y[i]});
        }
    }

    for (int i = 0; i < seeds.size(); ++i)
    {
        std::cout << "seeds[" << i << "]: " << seeds[i].x << ", " << seeds[i].y << std::endl;
    }

    //------------------------------------------------ CENTRALIZED COMPUTATION ---------------------------------------
    // only for visualization purposes

    //-----------------Voronoi--------------------

    //--------------------------------------------------------------------------------------------------------------

    bool robot_stopped = true;
    //-------------------------------------------- DISTRIBUTED COMPUTATION -------------------------------------
    //----------------------------------------------------------------
    // COVERAGE and VORONOI DIAGRAM COMPUTATION
    // local (distributed) coverage control of the robot
    // the robot position is known in a global reference system
    // the neighbors relative positions are known
    //
    // Single integrator dynamics:
    // xdot = K*(xc - x)
    // where: xc : x centroid, x : current position
    //----------------------------------------------------------------
    if (seeds.size() >= 1)
    {
        //-----------------Voronoi--------------------
        // Rielaborazione vettore "points" globale in coordinate locali
        auto local_seeds_i = reworkPointsVector(seeds, seeds[0]);
        // auto neighbor_seeds = this->erase_1st(seeds);

        // Filtering the seeds outside the box, (since the seeds received should be only the ones inside the sensing range
        //                                       no seeds should be filtered)
        auto flt_seeds = filterPointsVector(seeds, RangeBox);

        // the neighbor_seeds should be equal to the flt_seeds
        auto diagram = generateDecentralizedDiagram(flt_seeds, RangeBox, seeds[0], ROBOT_RANGE, AreaBox);

    // publish voronoi cell
                geometry_msgs::PolygonStamped polygon;
                polygon.header.frame_id = "world";
                polygon.header.stamp = ros::Time::now();
                geometry_msgs::Point32 _p;
                auto halfEdge = diagram.getFaces().at(0).outerComponent;
                do
                {
                    _p.x = halfEdge->origin->point.x+pose_x(0);
                    _p.y = halfEdge->origin->point.y+pose_y(0);
                    _p.z = 0.0;
                    polygon.polygon.points.push_back(_p);
                    halfEdge = halfEdge->next;                                                  //passaggio all'half-edge successivo della face 0
                } while(halfEdge != diagram.getFaces().at(0).outerComponent); 
            polyPub_.publish(polygon);


        if (GAUSSIAN_DISTRIBUTION)
        {
            // compute centroid -- GAUSSIAN DISTRIBUTION
            // centroid = computeGaussianCentroid(diagram, Vector2<double>{PT_X,PT_Y}, VAR);
            std::vector<double> VARs = {VAR};
            std::vector<Vector2<double>> MEANs = {{PT_X, PT_Y}};
            centroid = computePolygonCentroid(diagram, MEANs, VARs);
        }
        else
        {
            // compute centroid -- UNIFORM DISTRIBUTION
            centroid = computeCentroid(diagram);
        }
        visualization_msgs::Marker marker;
                marker.header.frame_id = "world";
                marker.header.stamp = ros::Time::now();
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.pose.position.x = centroid.x+pose_x(0);
                marker.pose.position.y = centroid.y+pose_y(0);
                marker.pose.position.z = 1.15;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.01;
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0;
                centrPub_.publish(marker);
    }

    //---------------------------------------------------------------------------------------
    // from centroids compute velocity vectors according to Lloyd law
    //---------------------------------------------------------------------------------------

    if (centroid.getNorm() > CONVERGENCE_TOLERANCE)
    {
        vel_x = K_gain * (centroid.x);
        vel_y = K_gain * (centroid.y);
        robot_stopped = false;
        if (vel_x > MAX_LIN_VEL)
        {
            vel_x = MAX_LIN_VEL;
        }
        else if (vel_x < -MAX_LIN_VEL)
        {
            vel_x = -MAX_LIN_VEL;
        }
        if (vel_y > MAX_LIN_VEL)
        {
            vel_y = MAX_LIN_VEL;
        }
        else if (vel_y < -MAX_LIN_VEL)
        {
            vel_y = -MAX_LIN_VEL;
        }
    }
    else
    {
        vel_x = 0;
        vel_y = 0;
        robot_stopped = false;
    }
    geometry_msgs::TwistStamped vel_msg;    
    if(use_cbf_){
    Eigen::Vector3d uopt;
    Eigen::Vector3d ustar(vel_x, vel_y, 0.0);
    Eigen::Vector3d p_i(pose_x(0), pose_y(0), pose_theta(0));
    Eigen::MatrixXd p_j(3, neighbours_num);
    Eigen::VectorXd h;
    
    std_msgs::Float64MultiArray h_msg;
    for (int j = 0; j<neighbours_num;j++){
        p_j.col(j) = Eigen::Vector3d({pose_x(j+1),pose_y(j+1),pose_theta(j+1)});
    }

    if (!fow_controller.applyCbf(uopt, h, ustar, p_i, neighbours_num, p_j))
    {
        vel_msg.header.stamp = ros::Time::now();
        vel_msg.twist.linear.x = uopt(0);
        vel_msg.twist.linear.y = uopt(1);
        vel_msg.twist.angular.z = uopt(2);
        // publish h function
        std::vector<double> h_help {h.data(), h.data() + h.size()};
        h_msg.data = h_help;
        hfuncPub_.publish(h_msg);
    }
    }
    else{
        vel_msg.header.stamp = ros::Time::now();
        vel_msg.twist.linear.x = vel_x;
        vel_msg.twist.linear.y = vel_y;
    }
    vel_msg.twist.linear.z = 1.0-pose_z(0);
    //-------------------------------------------------------------------------------------------------------
    // Compute velocities commands for the robot: differential drive control, for UAVs this is not necessary
    //-------------------------------------------------------------------------------------------------------
    // auto vel_msg = this->Diff_drive_compute_vel(vel_x, vel_y, this->pose_theta(0));
    
    //-------------------------------------------------------------------------------------------------------

    this->velPub_.publish(vel_msg);
#ifdef ARPL_
    mav_manager::Vec4::Request req;
    mav_manager::Vec4::Response res;
    req.goal[0] = vel_x;
    req.goal[1] = vel_y;
    req.goal[2] = 0;
    req.goal[3] = 0;
    sc_setDesVelInWorldFrame.call(req, res);
    if (res.success)
        std::cout << res.message << std::endl;

#endif

    //-------------------------------------------------------------------------------------------------------
    if (robot_stopped == true)
    {
        time(&this->timer_final_count);
        if (this->timer_final_count - this->timer_init_count >= shutdown_timer)
        {
            // shutdown node
            std::cout << "SHUTTING DOWN THE NODE" << std::endl;
            node_shutdown_request = 1;
            // this->stop();   //stop the controller and shutdown the node
        }
    }
    else
    {
        time(&this->timer_init_count);
    }
}

/*******************************************************************************
 * Main function
 *******************************************************************************/
// alternatively to a global variable to have access to the method you can make STATIC the class method interested,
// but some class function may not be accessed: "this->" method cannot be used

void nodeobj_wrapper_function(int)
{
    ROS_WARN("signal handler function CALLED");
    node_shutdown_request = 1;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "coverage_singlenode", ros::init_options::NoSigintHandler);
    signal(SIGINT, nodeobj_wrapper_function);

    // Controller node_controller;
    auto node_controller = std::make_shared<Controller>();

    while (!node_shutdown_request)
    {
        ros::spinOnce();
    }
    node_controller->stop();

    // ros::spin();
    // do pre-shutdown tasks
    if (ros::ok())
    {
        ROS_WARN("ROS HAS NOT BEEN PROPERLY SHUTDOWN, it is being shutdown again.");
        ros::shutdown();
    }

    return 0;
}
