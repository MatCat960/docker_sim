// STL
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
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
// SFML
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
// My includes
#include "FortuneAlgorithm.h"
#include "Voronoi.h"
#include "Graphics.h"
// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>

//------------------------------------------------------------------------
const bool centralized_centroids = false;   //compute centroids using centralized computed voronoi diagram, 
                                            //let this value true only for debug purposes := it will implement standard coverage
const float CONVERGENCE_TOLERANCE = 0.05;
//------------------------------------------------------------------------
const int debug = 0;
//------------------------------------------------------------------------
const int shutdown_timer = 5;           //count how many seconds to let the robots stopped before shutting down the node
//------------------------------------------------------------------------
const bool LOG_FILE_ON = false;
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

        //view graphical voronoi rapresentation - bool
        this->nh_priv_.getParam("GRAPHICS_ON", GRAPHICS_ON);
        //-----------------------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------- Subscribers and Publishers -----------------------------------------------------
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            // lambda function to pass multiple parameters to the callback -> I have a single definition for the callback and it creates multiple function poseCallback
            // TODONYU: change the topic name to match the one you use
            poseSub_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>("/cf" + std::to_string(ids_[i]) + "/cmd_vel", 1, std::bind(&Controller::poseCallback, this, i, std::placeholders::_1)));
            velPub_.push_back(nh_.advertise<geometry_msgs::Twist>("/vrpn_client_node/cf" + std::to_string(ids_[i]) + "/pose", 1));
        }
        controller_timer_ = nh_.createTimer(ros::Duration(0.1), std::bind(&Controller::Coverage, this));
        //------------------------------------------------------------------------------------------------------------------------------------

        //----------------------------------------------------------- init Variables ---------------------------------------------------------
        pose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
        pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
        time(&this->timer_init_count);
        time(&this->timer_final_count);
        //------------------------------------------------------------------------------------------------------------------------------------

        //----------------------------------------------- Graphics window -----------------------------------------------
        if (GRAPHICS_ON)
        {
            app_gui.reset(new Graphics{AREA_SIZE_x, AREA_SIZE_y, AREA_LEFT, AREA_BOTTOM, VAR});
        }
        //---------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------open log file -----------------------------------------------
        open_log_file();
        //---------------------------------------------------------------------------------------------------------------
    }
    ~Controller()
    {
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;
    }

    void stop();
    void poseCallback(int i, const geometry_msgs::PoseStamped::ConstPtr &msg);
    void Coverage();

    //Graphics -- draw coverage
    void check_window_event();

    //open write and close LOG file
    void open_log_file();
    void write_log_file(std::string text);
    void close_log_file();

private:
    std::vector<int> ids_{1,2,3};
    int ROBOTS_NUM = 3;
    double ROBOT_RANGE = 3;

    Eigen::VectorXd pose_x;
    Eigen::VectorXd pose_y;
    Eigen::VectorXd pose_theta;
    std::vector<Vector2<double>> seeds_xy;

    //------------------------------- Ros NodeHandle ------------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    //-----------------------------------------------------------------------------------
    //------------------------- Publishers and subscribers ------------------------------
    std::vector<ros::Publisher> velPub_;
    std::vector<ros::Subscriber> poseSub_;
    ros::Timer controller_timer_;
    //-----------------------------------------------------------------------------------

    //Rendering with SFML
    //------------------------------ graphics window -------------------------------------
    std::unique_ptr<Graphics> app_gui;
    //------------------------------------------------------------------------------------

    //---------------------------- Environment definition --------------------------------
    double AREA_SIZE_x = 2.0;
    double AREA_SIZE_y = 2.0;
    double AREA_LEFT = -1.0;
    double AREA_BOTTOM = -1.0;
    //------------------------------------------------------------------------------------

    //---------------------- Gaussian Density Function parameters ------------------------
    bool GAUSSIAN_DISTRIBUTION = false;
    double PT_X = 0.0;
    double PT_Y = 0.0;
    double VAR = 0.4;
    //------------------------------------------------------------------------------------

    //graphical view - ON/OFF
    bool GRAPHICS_ON = false;

    //timer - check how long robots are being stopped
    time_t timer_init_count;
    time_t timer_final_count;

    //ofstream on external log file
    std::ofstream log_file;
    long unsigned int log_line_counter=0;
};

void Controller::stop()
{
    //if (signum == SIGINT || signum == SIGKILL || signum ==  SIGQUIT || signum == SIGTERM)
    ROS_INFO("shutting down the controller, stopping the robots, closing the graphics window");
    if ((GRAPHICS_ON) && (this->app_gui->isOpen())){
        this->app_gui->close();
    }

    ros::Duration(0.1).sleep();

    geometry_msgs::Twist vel_msg;
    for (int i = 0; i < 100; ++i)
    {
        for (int r = 0; r < ROBOTS_NUM; ++r)
        {
            this->velPub_[r].publish(vel_msg);
        }
    }

    ROS_INFO("controller has been closed and robots have been stopped");
    this->close_log_file();
    ros::Duration(0.1).sleep();

    ros::shutdown();
}

void Controller::poseCallback(int i, const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    this->pose_x(i) = msg->pose.position.x;
    this->pose_y(i) = msg->pose.position.y;

    tf2::Quaternion q(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->pose_theta(i) = yaw;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------Rendering functions - for SFML graphical visualization-----------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Controller::check_window_event(){
    sf::Event event;
    if ((GRAPHICS_ON) && (this->app_gui->isOpen() && this->app_gui->window->pollEvent(event)))
    {
        if (event.type == sf::Event::Closed){
            this->app_gui->close();
        }
    }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Controller::Coverage(){
    auto start = ros::Time::now();
    //Parameters
    //double min_dist = 0.4;         //avoid robot collision
    int K_gain = 1;                  //Lloyd law gain
    this->write_log_file(std::to_string(this->log_line_counter) + "\t");
    this->log_line_counter = this->log_line_counter + 1;

    //Variables
    double vel_x=0, vel_y=0;
    std::vector<Vector2<double>> seeds;
    std::vector<Vector2<double>> centroids;
    Vector2<double> vel; Vector2<double> centroid;

    Box<double> AreaBox{AREA_LEFT, AREA_BOTTOM, AREA_SIZE_x + AREA_LEFT, AREA_SIZE_y + AREA_BOTTOM};
    Box<double> RangeBox{-ROBOT_RANGE, -ROBOT_RANGE, ROBOT_RANGE, ROBOT_RANGE};

    for (int i = 0; i < ROBOTS_NUM; ++i)
    {
        if ((this->pose_x(i) != 0.0) && (this->pose_y(i) != 0.0))
        {
            seeds.push_back({this->pose_x(i), this->pose_y(i)});
        }
        centroids.push_back({this->pose_x(i), this->pose_y(i)});
    }

    //------------------------------------------------ CENTRALIZED COMPUTATION ---------------------------------------
    //only for visualization purposes
    //-----------------Voronoi--------------------
    for (unsigned long s = 0; s < seeds.size(); ++s)
    {
        if(debug>=1){std::cout<<"seeds::"<<seeds[s].x<<" "<<seeds[s].y<<std::endl;};
    }

    if ((GRAPHICS_ON) && (this->app_gui->isOpen())){
        check_window_event();

        if (seeds.size() >= 2)
        {
            this->app_gui->clear();
            auto diagram = generateCentralizedDiagram(seeds, AreaBox);
            if (centralized_centroids){                         //flag x centralized centroids computation
                centroids = diagram.computeLloydRelaxation();
            }
            this->app_gui->drawDiagram(diagram);
            this->app_gui->drawPoints(diagram);
            this->app_gui->drawGlobalReference(sf::Color(255,255,0), sf::Color(255,255,255));
            //Display window
            this->app_gui->display();
        }
    }
    //--------------------------------------------------------------------------------------------------------------

    bool all_robots_stopped = true;

    for (int i = 0; i < ROBOTS_NUM; ++i)
    {
        //-------------------------------------------- DISTRIBUTED COMPUTATION -------------------------------------
        //----------------------------------------------------------------
        //COVERAGE and VORONOI DIAGRAM COMPUTATION
        //local (distributed) coverage control of the robot
        //the robot position is known in a global reference system
        //the neighbors relative positions are known
        //
        //Single integrator dynamics:
        //xdot = K*(xc - x)
        //where: xc : x centroid, x : current position
        //----------------------------------------------------------------

        if (!centralized_centroids)     //flag x distributed computation
        {
            if (seeds.size() >= 1)
            {
                //-----------------Voronoi--------------------
                //Elaborate the global coordinates in local coordinates (to virtually simulate a full distributed approach)
                auto local_seeds_i = reworkPointsVector(seeds, seeds[i]);
                //Filter points outside the sensing box (to virtually simulate a full distributed approach)
                auto flt_seeds = filterPointsVector(local_seeds_i, RangeBox);
                auto diagram = generateDecentralizedDiagram(flt_seeds, RangeBox, seeds[i], ROBOT_RANGE, AreaBox);

                if (GAUSSIAN_DISTRIBUTION)
                {
                    //compute centroid -- GAUSSIAN DISTRIBUTION
                    std::vector<double> VARs = {VAR};
                    std::vector<Vector2<double>> MEANs = {{PT_X, PT_Y}};
                    centroid = computePolygonCentroid(diagram, MEANs, VARs);
                } else {
                    //compute centroid -- UNIFORM DISTRIBUTION
                    centroid = computeCentroid(diagram);
                }
            }
        }

        //---------------------------------------------------------------------------------------
        // from centroids compute velocity vectors according to Lloyd law
        //---------------------------------------------------------------------------------------

        if (centralized_centroids)
        {
            Vector2<double> local_centroid = {(centroids[i].x - this->pose_x(i)), (centroids[i].y - this->pose_y(i))};
            if (local_centroid.getNorm() > CONVERGENCE_TOLERANCE)
            {
                vel_x = K_gain*(local_centroid.x);      //vel_x = -K_gain*(this->pose_x(i) - centroids[i].x);
                vel_y = K_gain*(local_centroid.y);      //vel_y = -K_gain*(this->pose_y(i) - centroids[i].y);
                all_robots_stopped = false;
            } else {
                vel_x = 0;
                vel_y = 0;
            }
            if (debug >= 2){std::cout<<"centroid position of " <<i<<"::"<<local_centroid<<std::endl;}

        } else {
            if (centroid.getNorm() > CONVERGENCE_TOLERANCE)
            {
                vel_x = K_gain*(centroid.x);
                vel_y = K_gain*(centroid.y);
                all_robots_stopped = false;
            } else {
                vel_x = 0;
                vel_y = 0;
                std::cout<<"-------------- Convergence reached!! --------------"<<std::endl;
            }
        }

        if (debug >= 2){std::cout<<"sending velocities to " << ids_[i] << ":: " << vel_x << ", "<<vel_y<<std::endl;}

        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = vel_x;
        vel_msg.linear.y = vel_y;
        this->velPub_[i].publish(vel_msg);
        //----------------------------------------------------------------------------------------------------------
        if (!centralized_centroids)
        {
            this->write_log_file(std::to_string(i) + "\t" + std::to_string(this->pose_x(i)) + "\t" + std::to_string(this->pose_y(i)) + "\t" + std::to_string(centroid.x) + "\t" + std::to_string(centroid.y) + "\t");
        }
    }
    if (all_robots_stopped == true)
    {
        time(&this->timer_final_count);
        if (this->timer_final_count - this->timer_init_count >= shutdown_timer)
        {
            //shutdown node
            std::cout<<"SHUTTING DOWN THE NODE"<<std::endl;
            node_shutdown_request = 1;
            //this->stop();   //stop the controller and shutdown the node
        }
    } else {
        time(&this->timer_init_count);
    }

    this->write_log_file("\n");
    //--------------------------------------------------------------------------------------------------------------
    auto end = ros::Time::now();
    std::cout<<"Computation time cost: -----------------: "<<end - start<<std::endl;
}

void Controller::open_log_file()
{
    if (LOG_FILE_ON)
    {
        std::time_t t = time(0);
        struct tm * now = localtime(&t);
        char buffer [80];

        char *dir = get_current_dir_name();
        std::string dir_str(dir);

        if (IsPathExist(dir_str + "/Coverage_exp_logs"))     //check if the folder exists
        {
            strftime (buffer,80,"/Coverage_exp_logs/%Y_%m_%d_%H-%M_logfile.txt",now);
        } else {
            system(("mkdir " + (dir_str + "/Coverage_exp_logs")).c_str());
            strftime (buffer,80,"/Coverage_exp_logs/%Y_%m_%d_%H-%M_logfile.txt",now);
        }

        std::cout<<"file name :: "<<dir_str + buffer<<std::endl;
        this->log_file.open(dir_str + buffer,std::ofstream::app);
    }
}

void Controller::write_log_file(std::string text)
{
    if (this->log_file.is_open())
    {
        this->log_file << text;
    }
}

void Controller::close_log_file()
{
    std::cout<<"Log file is being closed"<<std::endl;
    this->log_file.close();
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
    ros::init(argc, argv, "UAV_coverage", ros::init_options::NoSigintHandler);
    signal(SIGINT, nodeobj_wrapper_function);

    //Controller node_controller;
    auto node_controller = std::make_shared<Controller>();

    //ros::Rate loop_rate(10);
    //while (ros::ok())
    //{
    //node_controller->Coverage();
    //ros::spinOnce();
    //loop_rate.sleep();
    //}

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