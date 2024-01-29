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
// #include "Graphics.h"
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
    Controller() : nh_priv_("~")
    
    {
        ROS_INFO("Node Initialization");
        //-------------------------------------------------------- ROS parameters -----------------------------------------------------------
        this->nh_priv_.getParam("ROBOTS_NUM", ROBOTS_NUM);
        this->nh_priv_.getParam("MODEL_NAME", MODEL_NAME);

        // sensing range single robot (= halfe edge of the local sensing box)
        this->nh_priv_.getParam("ROBOT_RANGE", ROBOT_RANGE);

        // Parameters for Gaussian
        this->nh_priv_.getParam("GAUSSIAN_DISTRIBUTION", GAUSSIAN_DISTRIBUTION);
        this->nh_priv_.getParam("GAUSS_X", PT_X);
        this->nh_priv_.getParam("GAUSS_Y", PT_Y);
        this->nh_priv_.getParam("VAR", VAR);

        // Area parameter
        this->nh_priv_.getParam("AREA_SIZE_x", AREA_SIZE_x);
        this->nh_priv_.getParam("AREA_SIZE_y", AREA_SIZE_y);
        this->nh_priv_.getParam("AREA_LEFT", AREA_LEFT);
        this->nh_priv_.getParam("AREA_BOTTOM", AREA_BOTTOM);
        use_cbf_ = this->nh_priv_.param("use_cbf",true);

        // view graphical voronoi rapresentation - bool
        this->nh_priv_.getParam("GRAPHICS_ON", GRAPHICS_ON);

        this->nh_priv_.getParam("SAVE_POS", SAVE_POS);

        // Robot ID
        // this->nh_priv_.getParam("ROBOT_ID", ROBOT_ID);

        //-----------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------- Subscribers and Publishers ----------------------------------------------------
        // velPub_ = nh_.advertise<geometry_msgs::TwistStamped>("/turtlebot" + std::to_string(ROBOT_ID) + "/cmd_vel", 1);
        // poseSub_ = nh_.subscribe<geometry_msgs::PoseArray>("/supervisor/robot" + std::to_string(ROBOT_ID) + "/pose", 1, std::bind(&Controller::neighposeCallback, this, std::placeholders::_1));
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            poseSubs_.push_back(nh_.subscribe<nav_msgs::Odometry>("/turtlebot" + std::to_string(i) + "/odom", 100, std::bind(&Controller::odomCallback, this, std::placeholders::_1, i)));
            velPubs_.push_back(nh_.advertise<geometry_msgs::TwistStamped>("/turtlebot" + std::to_string(i) + "/cmd_vel", 1));
        }
        
        // std::string polyTemplate = "/turtlebot"+ std::to_string(ROBOT_ID) + "/voronoi_cell";
        // std::string centroidTemplate = "/turtlebot"+ std::to_string(ROBOT_ID) + "/voronoi_centroid";
  
        // voronoi cell publishers
        // polyPub_ = nh_.advertise<geometry_msgs::PolygonStamped>(polyTemplate,1);
        // voronoi centroid publishers
        // centrPub_ = nh_.advertise<visualization_msgs::Marker>(centroidTemplate,1);
        controller_timer_ = nh_.createTimer(ros::Duration(0.1), std::bind(&Controller::Coverage, this));
        // publish cbf h function
        
        //-----------------------------------------------------------------------------------------------------------------------------------
        //----------------------------------------------------------- init Variables ---------------------------------------------------------
        pose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_z = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
        time(&this->timer_init_count);
        time(&this->timer_final_count);

        if (SAVE_POS)
        {
            this->open_log_file();
        }
//------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------ARPL ROS SERVICE -----------------------------------------------
#ifdef ARPL_
        sc_setDesVelInWorldFrame = nh_.serviceClient<mav_manager::Vec4>("/" + MODEL_NAME + "/mav_services/setDesVelInWorldFrame");
#endif
        //---------------------------------------------------------------------------------------------------------------
        //----------------------------------------------- Graphics window -----------------------------------------------
        // if (GRAPHICS_ON)
        // {
        //     this->app_gui.reset(new Graphics{AREA_SIZE_x, AREA_SIZE_y, AREA_LEFT, AREA_BOTTOM, VAR});
        // }
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
    void neighposeCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg, int i);
    void Coverage();
    geometry_msgs::Twist Diff_drive_compute_vel(double vel_x, double vel_y, double alfa);
    std::vector<Vector2<double>> erase_1st(std::vector<Vector2<double>> input_vect);

    // Graphics -- draw coverage

    //open write and close LOG file
    void open_log_file();
    void write_log_file(std::string text);
    void close_log_file();

private:
    int ROBOTS_NUM = 6;
    int ROBOTS_MAX = 20;
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
    // fow_control::FowController fow_controller;
    int neighbours_num;
    //------------------------------- Ros NodeHandle ------------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    //-----------------------------------------------------------------------------------
    //------------------------- Publishers and subscribers ------------------------------
    ros::Publisher polyPub_, centrPub_,hfuncPub_;
    // ros::Subscriber neighposeSub_;
    std::vector<ros::Publisher> velPubs_;
    std::vector<ros::Subscriber> poseSubs_;
    ros::Timer controller_timer_;
    //-----------------------------------------------------------------------------------

    // Rendering with SFML
    //------------------------------ graphics window -------------------------------------
    bool GRAPHICS_ON = true;
    // std::unique_ptr<Graphics> app_gui;
    //------------------------------------------------------------------------------------
    bool SAVE_POS = false;
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
    double VAR = 2.0;
    //------------------------------------------------------------------------------------

    // graphical view - ON/OFF

    std::ofstream log_file;

    // timer - check how long robots are being stopped
    time_t timer_init_count;
    time_t timer_final_count;
#ifdef ARPL_
    ros::ServiceClient sc_setDesVelInWorldFrame;
#endif
};

bool IsPathExist(const std::string &s)
{
  struct stat buffer;
  return (stat (s.c_str(), &buffer) == 0);
}

void Controller::stop()
{
    // if (signum == SIGINT || signum == SIGKILL || signum ==  SIGQUIT || signum == SIGTERM)
    ROS_INFO("shutting down the controller, stopping the robots, closing the graphics window");

    ros::Duration(0.1).sleep();

    geometry_msgs::TwistStamped vel_msg;
    for (int r = 0; r < ROBOTS_NUM; ++r)
    {
        this->velPubs_[r].publish(vel_msg);
    }

    if (SAVE_POS)
    {
        std::cout << "Closing log file..." << std::endl;
        this->close_log_file();
    }

    ROS_INFO("controller has been closed and robots have been stopped");
    ros::Duration(0.1).sleep();

    ros::shutdown();
}

void Controller::open_log_file()
{
    std::time_t t = time(0);
    struct tm * now = localtime(&t);
    char buffer [80];

    char *dir = get_current_dir_name();
    std::string dir_str(dir);
    dir_str = dir_str + "/pf_logs";

    if (IsPathExist(dir_str))     //check if the folder exists
    {
        strftime (buffer,80,"/%Y_%m_%d_%H-%M_logfile.txt",now);
    } else {
        system(("mkdir " + (dir_str)).c_str());
        strftime (buffer,80,"/%Y_%m_%d_%H-%M_logfile.txt",now);
    }

    std::cout<<"file name :: "<<dir_str + buffer<<std::endl;
    this->log_file.open(dir_str + buffer,std::ofstream::app);
}

void Controller::write_log_file(std::string text)
{
    if (this->log_file.is_open())
    {
        // std::cout << text;
        this->log_file << text;
    }
}

void Controller::close_log_file()
{
    std::cout<<"Log file is being closed"<<std::endl;
    this->log_file.close();
}

void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr &msg, int i)
{
    // std::cout << "Sono in odomcallback" << std::endl;
    this->pose_x(i) = msg->pose.pose.position.x;
    this->pose_y(i) = msg->pose.pose.position.y;
    this->pose_z(i) = msg->pose.pose.position.z;
    this->pose_theta(i) = tf2::getYaw(msg->pose.pose.orientation);
}

void Controller::neighposeCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    pose_x = 100*Eigen::VectorXd::Ones(ROBOTS_NUM);
    pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
    pose_z = Eigen::VectorXd::Zero(ROBOTS_NUM);
    // pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
    for (unsigned int i = 0; i < msg->poses.size(); ++i)
    {
        auto x = msg->poses[i].position.x;
        auto y = msg->poses[i].position.y;
        auto z = msg->poses[i].position.z;
        auto theta = tf2::getYaw(msg->poses[i].orientation);

        std::cout << "robot -" << i << ": " << x << ", " << y << ", " << theta << std::endl;

        this->pose_x(i) = msg->poses[i].position.x;
        this->pose_y(i) = msg->poses[i].position.y;
        this->pose_z(i) = msg->poses[i].position.z;
        this->pose_theta(i) = tf2::getYaw(msg->poses[i].orientation);
    }
    neighbours_num = msg->poses.size()-1;
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
    // std::cout << "SOno nel loop" << std::endl;
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

    for (int i = 0; i < ROBOTS_NUM; ++i)
    {
        if (((this->pose_x(i)) != 0.0) && (this->pose_y(i) != 0.0))
        {
            seeds.push_back({this->pose_x(i), this->pose_y(i)});
        }
        // centroids.push_back({this->pose_x(i), this->pose_y(i)});
    }

    //------------------------------------------------ CENTRALIZED COMPUTATION ---------------------------------------
    // only for visualization purposes

    //-----------------Voronoi--------------------

    //--------------------------------------------------------------------------------------------------------------

    
    bool robots_stopped = true;
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
    bool write = true;
    std::string txt;
    for (int i = 0; i < ROBOTS_MAX; i++)
    {
        // std::cout << "Iteration number " << i << std::endl;
        if (i >= ROBOTS_NUM)
        {
            if (SAVE_POS)
            {
                txt += std::to_string(100.0) + " " + std::to_string(100.0) + "\n";
                txt += std::to_string(99.9) + " " + std::to_string(99.9) + "\n";
                // std::string txt = std::to_string(100.0) + " " + std::to_string(100.0) + "\n";
                // txt += std::to_string(99.9) + " " + std::to_string(99.9) + "\n";
                // this->write_log_file(txt);
            }
        } else
        {
            
            // std::cout << "Robot " << i << " position: " << this->pose_x(i) << ", " << this->pose_y(i) << std::endl;
            if (this->pose_x(i) == 0.0 && this->pose_y(i) == 0.0)
            {
                std::cout << "Robot " << i << " not detected" << std::endl;
                write = false;
                return;
            }
            
            if (SAVE_POS)
            {
                txt += std::to_string(this->pose_x(i)-PT_X) + " " + std::to_string(this->pose_y(i)-PT_Y) + "\n";
                // this->write_log_file(txt);
            }
            
            if (seeds.size() >= 1)
            {
                //-----------------Voronoi--------------------
                //Rielaborazione vettore "points" globale in coordinate locali
                auto local_seeds_i = reworkPointsVector(seeds, seeds[i]);
                
                //Filtraggio siti esterni alla box (simula azione del sensore)
                auto flt_seeds = filterPointsVector(local_seeds_i, RangeBox);
                auto diagram = generateDecentralizedDiagram(flt_seeds, RangeBox, seeds[i], ROBOT_RANGE, AreaBox);

                //compute centroid -- GAUSSIAN DISTRIBUTION
                std::vector<double> VARs = {VAR};
                std::vector<Vector2<double>> MEANs = {{PT_X, PT_Y}};
                centroid = computePolygonCentroid(diagram, MEANs, VARs);
                // std::cout << "Centroid: x = " << centroid[0] << ", y = " << centroid[1] <<std::endl;
            }
            
            //---------------------------------------------------------------------------------------
            // from centroids compute velocity vectors according to Lloyd law
            //---------------------------------------------------------------------------------------

            if (centroid.getNorm() > CONVERGENCE_TOLERANCE)
            {
                vel_x = K_gain * (centroid.x);
                vel_y = K_gain * (centroid.y);
                robots_stopped = false;
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
                // robots_stopped = false;
            }

            if (SAVE_POS)
            {
                txt += std::to_string(vel_x) + " " + std::to_string(vel_y) + "\n";
                // this->write_log_file(txt);
            }
            geometry_msgs::TwistStamped vel_msg;
            vel_msg.header.stamp = ros::Time::now();
            vel_msg.twist.linear.x = vel_x;
            vel_msg.twist.linear.y = vel_y;
            //-------------------------------------------------------------------------------------------------------
            // Compute velocities commands for the robot: differential drive control, for UAVs this is not necessary
            //-------------------------------------------------------------------------------------------------------
            // auto vel_msg = this->Diff_drive_compute_vel(vel_x, vel_y, this->pose_theta(0));
            
            //-------------------------------------------------------------------------------------------------------

            this->velPubs_[i].publish(vel_msg);
        }
    }

    if (SAVE_POS && write)
    {
        // std::cout << "Writing file..." << std::endl;
        this->write_log_file(txt);
    }

    // if ((GRAPHICS_ON) && (this->app_gui->isOpen()))
    // {
    //     // check_window_event();

    //     if (seeds.size() >= 2)
    //     {
    //         this->app_gui->clear();
    //         auto diagram = generateCentralizedDiagram(seeds, AreaBox);
    //         // if (centralized_centroids){                         //flag x centralized centroids computation
    //         //     centroids = diagram.computeLloydRelaxation();
    //         // }
    //         this->app_gui->drawDiagram(diagram);
    //         this->app_gui->drawPoints(diagram);
    //         this->app_gui->drawGlobalReference(sf::Color(255,255,0), sf::Color(255,255,255));
    //         // this->app_gui->drawGaussianContours(MEANs, VARs);
    //         // this->app_gui->drawGMM(this->gmm_msg);
    //         //Display window
    //         this->app_gui->display();
    //     }
    // }


    //-------------------------------------------------------------------------------------------------------
    if (robots_stopped == true)
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
