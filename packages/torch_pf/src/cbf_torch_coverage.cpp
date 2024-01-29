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
#include "FortuneAlgorithm.h"
#include "Voronoi.h"
#include "Diagram.h"
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

// Custom libraries
#include <fow_control/FowController.h>
#include <hqp/Hqp.h>

// Torch
# include <torch/script.h>      // One-stop header


#define M_PI   3.14159265358979323846  /*pi*/

using namespace std::chrono_literals;
using std::placeholders::_1;

//Robots parameters ------------------------------------------------------
double SAFETY_DIST = 2.0;
const double MAX_LIN_VEL = 1.0;         //set to turtlebot max velocities
// const double MAX_ANG_VEL = 2*M_PI*MAX_LIN_VEL/SAFETY_DIST;
const double MAX_ANG_VEL = 1.0;
const double b = 0.025;                 //for differential drive control (only if we are moving a differential drive robot (e.g. turtlebot))
//------------------------------------------------------------------------
const bool centralized_centroids = false;   //compute centroids using centralized computed voronoi diagram
const float CONVERGENCE_TOLERANCE = 0.1;
//------------------------------------------------------------------------
const int shutdown_timer = 15;           //count how many seconds to let the robots stopped before shutting down the node
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
    Controller() : nh_priv_("~"), fow_controller(2.0944, SAFETY_DIST, ROBOT_RANGE, ROBOTS_NUM-1), hqp_solver(2.0944, SAFETY_DIST, ROBOT_RANGE, ROBOTS_NUM-1)
    {
        //------------------------------------------------- ROS parameters ---------------------------------------------------------
        this->nh_priv_.getParam("ROBOTS_NUM", ROBOTS_NUM);

        // ID of the controlled robot
        this->nh_priv_.getParam("ROBOT_ID", ROBOT_ID);
        
        //Range di percezione singolo robot (= metà lato box locale)
        this->nh_priv_.getParam("ROBOT_RANGE", ROBOT_RANGE);
        this->nh_priv_.getParam("ROBOT_FOV", ROBOT_FOV);

        //view graphical voronoi rapresentation - bool
        this->nh_priv_.getParam("GRAPHICS_ON", GRAPHICS_ON);

        // Torchlib Models path
        this->nh_priv_.getParam("COVERAGE_MODEL_PATH", coverage_model_path);
        this->nh_priv_.getParam("PF_MODEL_PATH", pf_model_path);

        // Area parameter
        this->nh_priv_.getParam("AREA_SIZE_x", AREA_SIZE_x);
        this->nh_priv_.getParam("AREA_SIZE_y", AREA_SIZE_y);
        this->nh_priv_.getParam("AREA_LEFT", AREA_LEFT);
        this->nh_priv_.getParam("AREA_BOTTOM", AREA_BOTTOM);

        this->nh_priv_.getParam("GAUSSIAN_DISTRIBUTION", GAUSSIAN_DISTRIBUTION);
        this->nh_priv_.getParam("GAUSS_X", GAUSS_X);
        this->nh_priv_.getParam("GAUSS_Y", GAUSS_Y);
        
        this->nh_priv_.getParam("SAVE_LOGS", SAVE_LOGS);
        this->nh_priv_.getParam("SAVE_CPU_TIME", SAVE_CPU_TIME);

    //--------------------------------------------------- Subscribers and Publishers ----------------------------------------------------
    for (int i = 0; i < ROBOTS_NUM; i++)
    {   
        realposeSub_.push_back(nh_.subscribe<nav_msgs::Odometry>("/hummingbird" + std::to_string(i) + "/ground_truth/odometry", 1, std::bind(&Controller::realposeCallback, this, std::placeholders::_1, i)));
    }
    
    odomSub_ = nh_.subscribe<nav_msgs::Odometry>("/hummingbird" + std::to_string(ROBOT_ID) + "/ground_truth/odometry", 1, std::bind(&Controller::odomCallback, this, std::placeholders::_1));
    neighSub_ = nh_.subscribe<geometry_msgs::PoseArray>("/supervisor/robot" + std::to_string(ROBOT_ID) + "/pose", 1, std::bind(&Controller::neighCallback, this, std::placeholders::_1));
    velPub_.push_back(nh_.advertise<geometry_msgs::TwistStamped>("/hummingbird" + std::to_string(ROBOT_ID) + "/autopilot/velocity_command", 1));
    timer_ = nh_.createTimer(ros::Duration(0.25), std::bind(&Controller::cbf_control, this));
    
    //rclcpp::on_shutdown(std::bind(&Controller::stop,this));

    //----------------------------------------------------------- init Variables ---------------------------------------------------------
    pose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
    pose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
    pose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
    p_j.resize(3,ROBOTS_NUM);                           // matrix with global position of neighbors on each column
    p_j_i.resize(3, ROBOTS_NUM-1);
    p_j_est.resize(3, ROBOTS_NUM-1);
    p_j_est.setZero();
    covariances.resize(4, ROBOTS_NUM-1);                // matrix with covariance matrices for each neighbor
    covariances.setOnes();
    slack.resize(4, ROBOTS_NUM-1);
    slack_neg.resize(4, ROBOTS_NUM-1);
    realpose_x = Eigen::VectorXd::Zero(ROBOTS_NUM);
    realpose_y = Eigen::VectorXd::Zero(ROBOTS_NUM);
    realpose_theta = Eigen::VectorXd::Zero(ROBOTS_NUM);
    GAUSSIAN_MEAN_PT.resize(2);
    GAUSSIAN_MEAN_PT << GAUSS_X, GAUSS_Y;                       // Gaussian mean point
    time(&this->timer_init_count);
    time(&this->timer_final_count);

    this->got_gmm = false;

    slack_max.resize(4);
    double x1m, x2m, y1m, y2m;
    double fov = ROBOT_FOV * M_PI / 180;
    x1m = AREA_SIZE_x * cos(-fov/2 + 0.5*M_PI);
    x2m = AREA_SIZE_x * cos(fov/2 + 0.5*M_PI);
    y1m = AREA_SIZE_y * sin(-fov/2 + 0.5*M_PI);
    y2m = AREA_SIZE_y * sin(fov/2 + 0.5*M_PI);
    slack_max(0) = tan(fov/2) * x1m + y1m;
    slack_max(1) = tan(fov/2) * x2m - y2m;
    // slack_max(2) = -pow(SAFETY_DIST,2);
    slack_max(2) = 0.0;                         // safety constraint is always hard
    slack_max(3) = -(pow(AREA_SIZE_x,2) + pow(AREA_SIZE_y,2)) + pow(ROBOT_RANGE,2);
    slack_max = slack_max.cwiseAbs();
    std::cout << "============ SLACK SATURATION VALUES =================\n" << slack_max.transpose() << "\n==========================\n";

    fow_controller.setVerbose(false);
    // safety_controller.setVerbose(false);
    hqp_solver.setVerbose(false);

    // ---------------------- Load models ----------------------
    try
    {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        coverage_model = torch::jit::load(coverage_model_path);
        pf_model = torch::jit::load(pf_model_path);
        std::cout << "Models loaded correctly\n";
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    justStarted.resize(ROBOTS_NUM, true);
    // for (int i = 0; i < ROBOTS_NUM; i++)
    // {
    //     justStarted[i] = true;
    // }

	// this->got_gmm = false;
    std::cout << "Hi! I'm robot number " << ROBOT_ID << std::endl;
    
    
    // ----------------------------------------------- Graphics window -------------------------------------------------
    if (GRAPHICS_ON)
    {
        app_gui.reset(new Graphics{AREA_SIZE_x, AREA_SIZE_y, AREA_LEFT, AREA_BOTTOM, 2.0});
    }

    if (SAVE_LOGS || SAVE_CPU_TIME)
    {
        // for (int i = 0; i < ROBOTS_NUM - 1; i++)
        // {
        //     std::ofstream log_f;
        //     this->log_files.push_back(log_f);
        // }

        // fill vector with log files
        // this->log_files.resize(ROBOTS_NUM-1);
        // std::ofstream log_f;
        // for (int i = 0; i < ROBOTS_NUM-1; i++)
        // {
        //     this->log_files[i] = std::move(log_f);
        // }

        // for (int i = 0; i < ROBOTS_NUM-1; i++)
        // {
        //     open_log_file();
        // }
        this->open_log_file();
        std::string txt = "Ciao\n";
        this->write_log_file(txt); 
    }
    
    }
    ~Controller()
    {
        if ((GRAPHICS_ON || SAVE_CPU_TIME) && (this->app_gui->isOpen())){this->app_gui->close();}
        std::cout<<"DESTROYER HAS BEEN CALLED"<<std::endl;

        if((SAVE_LOGS || SAVE_CPU_TIME) && (this->log_file.is_open()))
        {
            // for (int i = 0; i < ROBOTS_NUM-1; i++)
            // {
            //     close_log_file(i);
            //     std::cout << "LOG FILE HAS BEEN CLOSED" << std::endl;
            // }
            this->close_log_file();
            
        }
    }

    //void stop(int signum);
    void stop();
    void realposeCallback(const nav_msgs::Odometry::ConstPtr msg, int i);
    void odomCallback(const nav_msgs::Odometry::ConstPtr msg);
    void neighCallback(const geometry_msgs::PoseArray::ConstPtr msg);
    bool insideFOV(Eigen::VectorXd q, Eigen::VectorXd q_obs, double fov, double r_sens);
    double sigmoid(double x);
    Eigen::VectorXd boundVel(Eigen::VectorXd u);
    void cbf_control();
    
    //open write and close LOG file
    void open_log_file();
    void write_log_file(std::string text);
    void close_log_file();



private:
    int ROBOTS_NUM = 16;
    int ROBOTS_MAX = 20;
    double ROBOT_RANGE = 15.0;
    int ROBOT_ID = 0;
    double ROBOT_FOV = 160.0;
    double GAUSS_X = 7.5;
    double GAUSS_Y = 7.5;
    double dist_lim = 1.0;
    bool got_gmm;
    double vel_linear_x, vel_angular_z;
    Eigen::VectorXd pose_x;
    Eigen::VectorXd pose_y;
    Eigen::VectorXd pose_theta;
    Eigen::MatrixXd p_j, p_j_i, p_j_est, slack, slack_neg;
    Eigen::MatrixXd covariances;
    Eigen::VectorXd slack_max;
    Eigen::VectorXd realpose_x;
    Eigen::VectorXd realpose_y;
    Eigen::VectorXd realpose_theta;
    std::vector<Vector2<double>> seeds_xy;
    int seeds_counter = 0;
    std::vector<double> position;
    double dt = 0.25;
    std::vector<bool> justLost;
    std::vector<bool> justStarted;

    //------------------------------- Ros NodeHandle ------------------------------------
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    //------------------------- Publishers and subscribers ------------------------------
    std::vector<ros::Publisher> velPub_;
    ros::Subscriber odomSub_;
    ros::Subscriber neighSub_;
    std::vector<ros::Subscriber> realposeSub_;
    ros::Timer timer_;

 
    // ------------------------------- FoV CBF ---------------------------------
    fow_control::FowController fow_controller;
    Eigen::VectorXd h, h_tmp;
    Eigen::Vector3d ustar;
    Eigen::Vector3d target;

    // ---- HQP solver -------
    Hqp hqp_solver;


    // ----------------- Libtorch models -----------------
    torch::jit::script::Module coverage_model;
    torch::jit::script::Module pf_model;
    std::string coverage_model_path = "/home/mattia/pf-training/SerializedModels/coverage_model_old.pt";
    std::string pf_model_path = "/home/mattia/pf-training/SerializedModels/pf_model_with_obs.pt";
    
    //Rendering with SFML
    //------------------------------ graphics window -------------------------------------
    std::unique_ptr<Graphics> app_gui;
    //------------------------------------------------------------------------------------

    //---------------------------- Environment definition --------------------------------
    double AREA_SIZE_x = 40.0;
    double AREA_SIZE_y = 40.0;
    double AREA_LEFT = -20.0;
    double AREA_BOTTOM = -20.0;
    //------------------------------------------------------------------------------------

    //---------------------- Gaussian Density Function parameters ------------------------
    bool GAUSSIAN_DISTRIBUTION = true;
    double PT_X;
    double PT_Y;
    double VAR;
    Eigen::VectorXd GAUSSIAN_MEAN_PT;

    //------------------------------------------------------------------------------------

    //graphical view - ON/OFF
    bool GRAPHICS_ON = true;

    bool SAVE_LOGS = false;
    bool SAVE_CPU_TIME = false;

    //timer - check how long robots are being stopped
    time_t timer_init_count;
    time_t timer_final_count;

    std::ofstream log_file;
    std::vector<std::ofstream> log_files;

    int counter = 0;

};

void Controller::open_log_file()
{
    std::time_t t = time(0);
    struct tm * now = localtime(&t);
    char buffer [80];

    char *dir = get_current_dir_name();
    std::string dir_str(dir);

    std::cout << "Directory: " << dir_str << std::endl;
    std::string dir_path = dir_str + "/pf_logs-" + std::to_string(ROBOT_ID); //+"-"+std::to_string(id);
    if (IsPathExist(dir_path))     //check if the folder exists
    {
        strftime (buffer,80,"/%Y_%m_%d_%H-%M_logfile.txt", now);
    } else {
        system(("mkdir "+ dir_path).c_str()); //+"-"+std::to_string(id)).c_str());
        strftime (buffer,80,"/%Y_%m_%d_%H-%M_logfile.txt",now);
    }

    std::cout<<"file name :: "<<dir_path + buffer<<std::endl;
    // this->log_file.open(dir_path + buffer,std::ofstream::app);
    this->log_file.open(dir_path + buffer,std::ofstream::app);
    // this->log_file << "Robot " << ROBOT_ID << " log file\n";
}

void Controller::write_log_file(std::string text)
{
    if (this->log_file.is_open())
    {
        this->log_file << text;
        this->log_file.flush();
    }
}

void Controller::close_log_file()
{
    std::cout<<"Log file is being closed"<<std::endl;
    this->log_file.close();
}

void Controller::stop()
{
    //if (signum == SIGINT || signum == SIGKILL || signum ==  SIGQUIT || signum == SIGTERM)
    ROS_INFO("shutting down the controller, stopping the robots, closing the graphics window");
    if ((GRAPHICS_ON) && (this->app_gui->isOpen())){
        this->app_gui->close();
    }

    if ((SAVE_LOGS || SAVE_CPU_TIME) && (this->log_file.is_open()))
    {
        this->close_log_file();   
    }
    // this->timer_->cancel();
    ros::Duration(0.1).sleep();

    geometry_msgs::TwistStamped vel_msg;
    for (int i = 0; i < 100; ++i)
    {
        this->velPub_[0].publish(vel_msg);
    }

    ROS_INFO("controller has been closed and robots have been stopped");
    ros::Duration(0.1).sleep();

    ros::shutdown();
}

void Controller::realposeCallback(const nav_msgs::Odometry::ConstPtr msg, int i)
{
    this->realpose_x(i) = msg->pose.pose.position.x;
    this->realpose_y(i) = msg->pose.pose.position.y;
    
    tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->realpose_theta(i) = yaw;

    if (this->justStarted[i])
    {
        this->pose_x(i) = this->realpose_x(i);
        this->pose_y(i) = this->realpose_y(i);
        this->pose_theta(i) = this->realpose_theta(i);
        this->justStarted[i] = false;
    }
}

void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr msg)
{
    this->pose_x(ROBOT_ID) = msg->pose.pose.position.x;
    this->pose_y(ROBOT_ID) = msg->pose.pose.position.y;

    tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    this->pose_theta(ROBOT_ID) = yaw;

    // std::cout << "I'm in odomCallback" << "\n";
    // std::cout << "Robot position: " << this->pose_x(ROBOT_ID) << ", " << this->pose_y(ROBOT_ID) << ", " << this->pose_theta(ROBOT_ID) << "\n";
}

void Controller::neighCallback(const geometry_msgs::PoseArray::ConstPtr msg)
{
    for (int j = 0; j < ROBOTS_NUM; j++)
    {
        if (j != ROBOT_ID)
        {
            this->pose_x(j) = msg->poses[j].position.x;
            this->pose_y(j) = msg->poses[j].position.y;

            tf2::Quaternion q(
            msg->poses[j].orientation.x,
            msg->poses[j].orientation.y,
            msg->poses[j].orientation.z,
            msg->poses[j].orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            if (!isnan(yaw))
            {
                this->pose_theta(j) = yaw;
            } else
            {
                this->pose_theta(j) = M_PI;
            }

            // Conversion to global position
            Eigen::MatrixXd R_w_i;                          // rotation matrix from local to global
            // R_w_i.resize(3,3);
            // R_w_i << cos(this->pose_theta(ROBOT_ID)), -sin(this->pose_theta(ROBOT_ID)), 0,
            //         sin(this->pose_theta(ROBOT_ID)), cos(this->pose_theta(ROBOT_ID)), 0,
            //         0, 0, 1;
            p_j(0,j) = this->pose_x(ROBOT_ID) + this->pose_x(j) * cos(this->pose_theta(ROBOT_ID)) - this->pose_y(j) * sin(this->pose_theta(ROBOT_ID));
            p_j(1,j) = this->pose_y(ROBOT_ID) + this->pose_x(j) * sin(this->pose_theta(ROBOT_ID)) + this->pose_y(j) * cos(this->pose_theta(ROBOT_ID));
            p_j(2,j) = this->pose_theta(j);

            int c = j;
            if (j>ROBOT_ID) {c = j-1;}
            p_j_i.col(c) << this->pose_x(j), this->pose_y(j), this->pose_theta(j);

            if (msg->poses[j].position.x != 100.0 && msg->poses[j].position.y != 100.0)
            {
                p_j_est(0,c) = p_j(0,j);
                p_j_est(1,c) = p_j(1,j);
                p_j_est(2,c) = this->pose_theta(j);
            }
        } else
        {
            // column of the controlled robot contains its own global position
            p_j.col(j) << this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID), this->pose_theta(ROBOT_ID);
        }

        
    }

    // std::cout << "Global position of robots: \n" << p_j << std::endl;
}

bool Controller::insideFOV(Eigen::VectorXd q, Eigen::VectorXd q_obs, double fov, double r_sens)
{
    /* ----------------------------------------------
    Check if q_obs is inside the field of view of q
    q = [x, y, th] : robot pose
    q_obs = [x, y] : neighbor pose (orientation ignored)
    fov = field of view [rad]
    r_sens = sensing range [m]
    
    th_obs = neighbor bearing wrt [0,0,0]
    th_diff = neighbor bearing wrt robot reference frame
    -------------------------------------------------*/

    double dist = sqrt(pow(q_obs(0)-q(0),2) + pow(q_obs(1)-q(1),2));
    double th = q(2);
    double th_obs = atan2(q_obs(1)-q(1), q_obs(0)-q(0));
    double th_rel = th_obs - th;

    double fov_rad = fov*M_PI/180;

    if (th_rel > M_PI)
        th_rel -= 2*M_PI;
    else if (th_rel < -M_PI)
        th_rel += 2*M_PI;

    if (th_rel > 0.5*fov_rad || th_rel < -0.5*fov_rad || dist > r_sens)
        return false;
    else
        return true;
}

double Controller::sigmoid(double x)
{
    double v = 1 / (1 + exp(-x));
    return v;
}


void Controller::cbf_control()
{
    slack.setOnes();
    slack = 100000*slack;
    slack.row(2).setZero();
    auto timerstart = std::chrono::high_resolution_clock::now();
    // double timerstart = ros::Time::now().toSec();
    
    // Check robot's initialization
    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        if (this->pose_x(i) == 0.0 && this->pose_y(i) == 0.0)
        {
            std::cout << "Error in robot " << i << " initialization. Skipping..." << std::endl;
            return;
        }
    }
    Eigen::Vector3d robot;
    robot << this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID), this->pose_theta(ROBOT_ID);

    // Define rotation matrix
    Eigen::Matrix<double,3,3> R_w_i;
    R_w_i << cos(this->pose_theta(ROBOT_ID)), -sin(this->pose_theta(ROBOT_ID)), 0,
             sin(this->pose_theta(ROBOT_ID)), cos(this->pose_theta(ROBOT_ID)), 0,
             0, 0, 1;
        

    if (SAVE_LOGS)
    {
        std::string txt;
        // txt = txt + std::to_string(this->pose_x(ROBOT_ID)) + " " + std::to_string(this->pose_y(ROBOT_ID)) + "\n";
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            txt = txt + std::to_string(this->realpose_x(i)-GAUSS_X) + " " + std::to_string(this->realpose_y(i)-GAUSS_Y) + "\n";
        }
        this->write_log_file(txt);
    }

    // Predict next position of each robot
    std::vector<torch::jit::IValue> cov_inputs;
    at::Tensor robots = torch::zeros({ROBOTS_MAX, 2});
    auto a = robots.accessor<float, 2>();
    // std::cout << "p_j_est: " << p_j_est << std::endl;
    if ((p_j_est.array() == 0.0).all())
    {
        std::cout << "Null elements found in p_j_est. Skipping..." << std::endl;
        return;
    }
    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        int c = i;
        if (i >= ROBOT_ID) {c = i-1;}
        if (i == ROBOT_ID)
        {
            a[i][0] = this->pose_x(ROBOT_ID) - GAUSS_X;
            a[i][1] = this->pose_y(ROBOT_ID) - GAUSS_Y;
        } else
        {
            a[i][0] = p_j_est(0, c) - GAUSS_X;
            a[i][1] = p_j_est(1, c) - GAUSS_Y;
        }
        
    }
    robots = robots.view({-1, 2*ROBOTS_MAX});
    // std::cout << "Robots input : " << robots << std::endl;
    cov_inputs.push_back(robots);
    // auto pfstart = std::chrono::high_resolution_clock::now();
    at::Tensor cov_output = this->coverage_model.forward(cov_inputs).toTensor();
    // auto pfend = std::chrono::high_resolution_clock::now();
    // std::cout << "Coverage inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(pfend-pfstart).count() << " ms\n";
    // std::cout << "Predicted velocity: " << cov_output << "\n";
    
    // Fill p_j_est with output
    auto b = cov_output.accessor<float, 2>();
    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        int c = i;
        if (i >= ROBOT_ID) {c = i-1;}
        if (i != ROBOT_ID)
        {
            // Replace values for undetected robot with predicted ones
            if (this->pose_x(i) == 100.0 && this->pose_y(i) == 100.0)
            {
                p_j_est(0,c) = p_j_est(0,c) + dt * 0.5 * b[0][2*i];                         // use half of predicted vel
                p_j_est(1,c) = p_j_est(1,c) + dt * 0.5 * b[0][2*i+1];
                p_j_est(2,c) = this->pose_theta(i);
            }
        }
    }

    // std::cout << "Predicted positions: \n" << p_j_est << "\n";

    // ----------- AT THIS POINT I SHOULD HAVE p_j_est FILLED WITH DETECTIONS OR PREDICTIONS -------------

    // Transform output to local frame
    Eigen::MatrixXd p_j_est_local = Eigen::MatrixXd::Zero(3, ROBOTS_NUM-1);
    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        int c = i;
        if (i >= ROBOT_ID) {c = i-1;}
        if (i != ROBOT_ID)
        {
            p_j_est_local.col(c) = R_w_i.transpose() * (p_j_est.col(c) - robot);
        }
    }

    // Get detections and create input for PF model
    /*
    std::vector<std::vector<torch::jit::IValue>> pf_inputs_total;
    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        int c = i;
        if (i >= ROBOT_ID) {c = i-1;}
        if (i != ROBOT_ID)
        {
            // Create input
            std::vector<torch::jit::IValue> pf_input_single;
            at::Tensor obs = torch::zeros({1, 8});
            auto obs_accessor = obs.accessor<float, 2>();

            // Fill 1st part of the input tensor with robot position and covariance at t-1
            obs_accessor[0][0] = p_j_est_local(0,c);
            obs_accessor[0][1] = p_j_est_local(1,c);
            obs_accessor[0][2] = covariances(0,c);
            obs_accessor[0][3] = covariances(1,c);
            obs_accessor[0][4] = covariances(2,c);
            obs_accessor[0][5] = covariances(3,c);

            // Case 1: robot detected -> fill 2nd part of the input tensor with detection
            if (this->pose_x(i) != 100.0 && this->pose_y(i) != 100.0)
            {
                obs_accessor[0][6] = p_j_est_local(0,c);
                obs_accessor[0][7] = p_j_est_local(1,c);
            }

            // Case 2: robot not detected -> 2nd part of the input tensor is left with zeros
            

            // Push back input
            // std::cout << "Obs " << std::to_string(i) << " : " << obs << "\n";
            pf_input_single.push_back(obs);
            pf_inputs_total.push_back(pf_input_single);
        }
    }
    */

    std::vector<torch::jit::IValue> pf_inputs_total;
    at::Tensor obs = torch::zeros({ROBOTS_MAX, 8});
    auto obs_accessor = obs.accessor<float, 2>();
    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        int c = i;
        if (i >= ROBOT_ID) {c = i-1;}
        if (i != ROBOT_ID)
        {
            // Fill 1st part of the input tensor with robot position and covariance at t-1
            obs_accessor[i][0] = p_j_est_local(0,c);
            obs_accessor[i][1] = p_j_est_local(1,c);
            obs_accessor[i][2] = covariances(0,c);
            obs_accessor[i][3] = covariances(1,c);
            obs_accessor[i][4] = covariances(2,c);
            obs_accessor[i][5] = covariances(3,c);

            // Case 1: robot detected -> fill 2nd part of the input tensor with detection
            if (this->pose_x(i) != 100.0 && this->pose_y(i) != 100.0)
            {
                obs_accessor[i][6] = p_j_est_local(0,c);
                obs_accessor[i][7] = p_j_est_local(1,c);
            }

            // Case 2: robot not detected -> 2nd part of the input tensor is left with zeros
        }
    }

    pf_inputs_total.push_back(obs);
    // std::cout << "PF inputs created\n";





    // std::cout << "Size of pf_inputs_tot: " << pf_inputs_total.size() << "\n";
    // auto pfstart2 = std::chrono::high_resolution_clock::now();
    // at::Tensor tot_output = this->pf_model.forward(pf_inputs_total).toTensor();
    // auto pfend2 = std::chrono::high_resolution_clock::now();
    // std::cout << "Collective PF inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(pfend2-pfstart2).count() << " ms\n";

    /*
    // Pass inputs to PF model and save covariance matrix
    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        int c = i;
        if (i >= ROBOT_ID) {c = i-1;}
        if (i != ROBOT_ID)
        {
            // auto pfstart3 = std::chrono::high_resolution_clock::now();
            at::Tensor single_output = this->pf_model.forward(pf_inputs_total[c]).toTensor();
            // auto pfend3 = std::chrono::high_resolution_clock::now();
            // std::cout << "PF inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(pfend3-pfstart3).count() << " ms\n";
            // std::cout << "Output of PF model for robot " << std::to_string(i) << " : " << single_output << "\n";
            auto single_output_accessor = single_output.accessor<float, 2>();
            covariances(0,c) = single_output_accessor[0][2];
            covariances(1,c) = single_output_accessor[0][3];
            covariances(2,c) = single_output_accessor[0][4];
            covariances(3,c) = single_output_accessor[0][5];
            // std::cout << "Covariance matrix for robot " << std::to_string(i) << ": \n" << covariances.col(c).transpose() << "\n";
        }
    }
    */

    // auto pfstart3 = std::chrono::high_resolution_clock::now();
    at::Tensor pf_output = this->pf_model.forward(pf_inputs_total).toTensor();
    // auto pfend3 = std::chrono::high_resolution_clock::now();
    // std::cout << "PF inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(pfend3-pfstart3).count() << " ms\n";
    // auto tot_time = std::chrono::duration_cast<std::chrono::milliseconds>(pfend3-pfstart + pfend-pfstart).count();
    // std::cout << "Total inference time: " << tot_time << " ms\n";
    // if (SAVE_CPU_TIME)
    // {
    //     std::string txt = std::to_string(tot_time) + "\n";
    //     this->write_log_file(txt);
    // }

    auto pf_output_accessor = pf_output.accessor<float, 2>();
    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        int c = i;
        if (i >= ROBOT_ID) {c = i-1;}
        if (i != ROBOT_ID)
        {
            covariances(0,c) = pf_output_accessor[i][2];
            covariances(1,c) = pf_output_accessor[i][3];
            covariances(2,c) = pf_output_accessor[i][4];
            covariances(3,c) = pf_output_accessor[i][5];
        }
    }

    if ((GRAPHICS_ON) && (this->app_gui->isOpen()))
    {
        this->app_gui->clear();
        this->app_gui->drawGlobalReference(sf::Color(255,255,0), sf::Color(255,255,255));
        this->app_gui->drawFOV(robot, ROBOT_FOV, ROBOT_RANGE);
        this->app_gui->drawPoint(GAUSSIAN_MEAN_PT, sf::Color(255,128,0));
        for (int i = 0; i < ROBOTS_NUM; i++)
        {
            auto color = sf::Color(0,255,0);                        // default color for other robots: green
            if (i == ROBOT_ID) {color = sf::Color(255,0,0);}        // controlled robot color: red
            Vector2<double> n;
            n.x = this->realpose_x(i);
            n.y = this->realpose_y(i);
            this->app_gui->drawPoint(n, color);
            // this->app_gui->drawID(n, i, color);
        }

        for (int j = 0; j < ROBOTS_NUM-1; j++)
        {
            Vector2<double> mp = {p_j_est(0,j), p_j_est(1,j)};
            this->app_gui->drawPoint(mp, sf::Color(0,0,255));
        }
    }

    // Compute and draw ellipses
    std::vector<double> distances(ROBOTS_NUM-1);
    for (int i = 0; i < ROBOTS_NUM; i++)
    {
        int c = i;
        if (i >= ROBOT_ID) {c = i-1;}
        if (i != ROBOT_ID)
        {
            Eigen::VectorXd pj = p_j_est.col(c).head(2);
            Eigen::Matrix2d cov;
            cov << covariances(0,c), covariances(1,c),
                    covariances(2,c), covariances(3,c);
            Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
            Eigen::Vector2d eigenvalues = es.eigenvalues().real();
            Eigen::Matrix2d eigenvectors = es.eigenvectors().real();

            // s = 4.605 for 90% confidence interval
            // s = 5.991 for 95% confidence interval
            // s = 9.210 for 99% confidence interval
            double s = 4.605;
            double a = sqrt(s*eigenvalues(0));            // major axis
            double b = sqrt(s*eigenvalues(1));            // minor axis

            // a could be smaller than b, so swap them
            if (a < b)
            {
                double temp = a;
                a = b;
                b = temp;
            }

            int m = 0;                  // higher eigenvalue index
            int l = 1;                  // lower eigenvalue index
            if (eigenvalues(1) > eigenvalues(0)) 
            {
                m = 1;
                l = 0;
            }
            
            double theta = atan2(eigenvectors(1,m), eigenvectors(0,m));             // angle of the major axis wrt positive x-asis (ccw rotation)
            if (theta < 0.0) {theta += M_PI;}                                    // angle in [0, 2pi
            if ((GRAPHICS_ON) && (this->app_gui->isOpen()))
            {
                this->app_gui->drawEllipse(pj, a, b, theta);
            }

            double slope = atan2(-pj(1) + this->pose_y(ROBOT_ID), -pj(0) + this->pose_x(ROBOT_ID));
            double x_n = pj(0) + a * cos(slope - theta) * cos(theta) - b * sin(slope - theta) * sin(theta);
            double y_n = pj(1) + a * cos(slope - theta) * sin(theta) + b * sin(slope - theta) * cos(theta);
            Vector2<double> p_near = {x_n, y_n};
            if (isnan(p_near.x) || isnan(p_near.y))
            {
                // ROS_WARN("NaN distance calculated");
                // std::cout << "Ellipses axis: " << a << " - " << b << std::endl;
                // std::cout << "Cov matrix values: " << cov << std::endl;
                p_near = {pj(0), pj(1)};

            }

            if (GRAPHICS_ON && this->app_gui->isOpen())
            {
                this->app_gui->drawPoint(p_near, sf::Color(255,102,255));
            }

            double dist = sqrt(pow(p_near.x - this->pose_x(ROBOT_ID), 2) + pow(p_near.y - this->pose_y(ROBOT_ID), 2));
            // if (isnan(dist))
            // {
            //     dist = 5.0;
            // }

            // Check if robot is inside ellipse
            double d = sqrt(pow(pj(0) - this->pose_x(ROBOT_ID), 2) + pow(pj(1) - this->pose_y(ROBOT_ID), 2));
            double range = sqrt(pow(pj(0) - p_near.x, 2) + pow(pj(1) - p_near.y, 2));
            if (d < range)
            {
                distances[c] = -dist;
            } else
            {
                distances[c] = dist;
            }

        }
    }

    // std::cout << "Distances : \n";
    // for (int i = 0; i < distances.size(); i++)
    // {
    //     std::cout << distances[i] << " ";
    //     slack.col(i) =  slack_max.cwiseProduct(sigmoid(distances[i] - 3*dist_lim) * Eigen::VectorXd::Ones(4)); // + slack_neg.col(i);
    // }
    // std::cout << std::endl;


    // --------------------- HQP SLACK VARIABLES CALCULATION ----------------------------------------
    std::vector<Eigen::VectorXd> p_j_ordered;
    // p_j_ordered.resize(2, ROBOTS_NUM-1);
    
    std::vector<std::pair<Eigen::Vector3d, double>> pos_pairs;
    for (int i = 0; i < distances.size(); i++)
    {
        // std::cout << p_j_est.col(i).transpose() << "\n";
        pos_pairs.push_back(std::make_pair(p_j_est.col(i), distances[i]));
    }
    
    // sort the vector based on distances
    std::sort(pos_pairs.begin(), pos_pairs.end(), [](const auto& a, const auto& b){
        return a.second < b.second;
    });

    std::vector<Eigen::VectorXd> slack_ordered;
    // slack_ordered.resize(2, ROBOTS_NUM-1);
    
    std::vector<std::pair<Eigen::VectorXd, double>> slack_pairs;
    for (int i = 0; i < distances.size(); i++)
    {
        slack_pairs.push_back(std::make_pair(slack.col(i), distances[i]));
    }

    // sort the vector based on distances
    std::sort(slack_pairs.begin(), slack_pairs.end(), [](const auto& a, const auto& b){
        return a.second < b.second;
    });

    for (int i = 0; i < distances.size(); i++)
    {
        p_j_ordered.push_back(pos_pairs[i].first);
        slack_ordered.push_back(slack_pairs[i].first);
    }

    // calculate hqp slack variable
    Eigen::VectorXd hqp_slack = hqp_solver.solve(p_j_ordered);
    // std::cout << "HQP slack: \n" << hqp_slack.transpose() << "\n";

    // backwards conversion to Eigen::MatrixXd
    Eigen::MatrixXd p_j_mat, slack_mat;
    p_j_mat.resize(3, ROBOTS_NUM-1);
    slack_mat.resize(4, ROBOTS_NUM-1);
    for (int i = 0; i < distances.size(); i++)
    {
        double dx = p_j_ordered[i](0) - this->pose_x(ROBOT_ID);
        double dy = p_j_ordered[i](1) - this->pose_y(ROBOT_ID);
        Eigen::Vector3d p_local = {dx, dy, p_j_ordered[i](2)};
        p_local = R_w_i.transpose() * p_local;
        p_j_mat.col(i) = p_local;
        slack_mat.col(i) = slack_ordered[i] + hqp_slack.block<4,1>(4*i,0);
    }
    //---------------------------------------------------------------------------

    // std::cout << "Starting Voronoi calculation... \n";
    // ------------------------------- Voronoi -------------------------------
    // Get detected or estimated position of neighbors in local coordinates
    Box<double> AreaBox{AREA_LEFT, AREA_BOTTOM, AREA_SIZE_x + AREA_LEFT, AREA_SIZE_y + AREA_BOTTOM};
    Box<double> RangeBox{ROBOT_RANGE, ROBOT_RANGE, ROBOT_RANGE, ROBOT_RANGE};
    std::vector<double> VARs = {1.0};
    std::vector<Vector2<double>> MEANs = {{GAUSSIAN_MEAN_PT(0), GAUSSIAN_MEAN_PT(1)}};
    double vel_x=0, vel_y=0;

    Vector2<double> p = {this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID)};
    std::vector<Vector2<double>> local_points;
    local_points.push_back(p);
    // Vector2<double> p_local = {0.0, 0.0};
    // local_points.push_back(p_local);
    for (int i = 0; i < ROBOTS_NUM-1; i++)
    {
        Vector2<double> p_local = {p_j_est(0,i) - p.x, p_j_est(1,i) - p.y};
        local_points.push_back(p_local);
    }

    // std::cout << "Generating decentralized diagram.\n";
    auto diagram = generateDecentralizedDiagram(local_points, RangeBox, p, ROBOT_RANGE, AreaBox);
    // std::cout << "Diagram generated. Calculating centroid.\n";
    Vector2<double> centroid;
    if (this->GAUSSIAN_DISTRIBUTION)
    {
        centroid = computePolygonCentroid(diagram, MEANs, VARs);
    } else
    {
        centroid = calculateCentroid(diagram);
    }
    // std::cout << "Centroid: " << centroid.x << ", " << centroid.y << "\n";
    Eigen::Vector2d centroid_eigen = {this->pose_x(ROBOT_ID)+centroid.x, this->pose_y(ROBOT_ID)+centroid.y};
    Eigen::Vector2d centroid_local = R_w_i.block<2,2>(0,0).transpose() * (centroid_eigen - robot.head(2));
 
    
    Eigen::Vector3d udes;
    double head_des = atan2(centroid_eigen(1), centroid_eigen(0));
    double head_err = head_des - this->pose_theta(ROBOT_ID);
    udes(0) = 0.8 * centroid.x;
    udes(1) = 0.8 * centroid.y;
    udes(2) = head_err;

    udes = boundVel(udes);


    Eigen::Vector3d uopt, uopt_loc, utemp, utemp_loc;
    Eigen::Vector3d udes_loc = R_w_i.transpose() * udes;
    // std::cout << "Desired local velocity for robot " << this->ROBOT_ID <<": " << udes_loc.transpose() << "\n";

    // std::cout << "Slack variables: \n" << slack_mat << std::endl;
    // std::cout << "p_j_mat: \n" << p_j_mat << std::endl;

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! p_j_est !!!!!!!!!!!!!!!!!!!
    if (!fow_controller.applyCbfSingle(utemp_loc, h_tmp, udes_loc, robot, ROBOTS_NUM-1, p_j_mat, slack_mat))              // slack variables equal to 0 in dangerous conditions -> hard constraint 
    {
        // std::cout << "h value: \n" << h_tmp.transpose() << std::endl;
        // utemp = R_w_i.transpose() * utemp_loc;
        utemp = R_w_i * utemp_loc;
        // std::cout << "Desired control input: " << udes.transpose() << "\n";

        // uopt = utemp;
        uopt = boundVel(utemp);
        // std::cout << "optimal control input: " << uopt.transpose() << std::endl;
    } else
    {
        ROS_WARN("SAFETY CBF FAILED");
        uopt = udes;
    }


    
    // std::cout << "Optimal control input: " << uopt.transpose() << "\n";
    // // utemp = boundVel(utemp);
    

    if (uopt.head(2).norm() < CONVERGENCE_TOLERANCE)
    {
        std::cout << "Converged\n";
        uopt.head(2).setZero();
    }

    // Publish control input
    geometry_msgs::TwistStamped vel_msg;
    // vel_msg.header.frame_id = "/hummingbird" + std::to_string(ROBOT_ID) + "/base_link";
    vel_msg.twist.linear.x = uopt(0);
    vel_msg.twist.linear.y = uopt(1);
    vel_msg.twist.angular.z = uopt(2);
    this->velPub_[0].publish(vel_msg);

    if ((GRAPHICS_ON) && (this->app_gui->isOpen()))
    {
        // Draw Voronoi diagram (centralized)
        std::vector<Vector2<double>> mean_points_vec2;
        Vector2<double> n = {this->pose_x(ROBOT_ID), this->pose_y(ROBOT_ID)};
        mean_points_vec2.push_back(n);
        for (int j = 0; j < ROBOTS_NUM-1; j++)
        {
            Vector2<double> mp = {p_j_est(0,j), p_j_est(1,j)};
            mean_points_vec2.push_back(mp);
            this->app_gui->drawPoint(mp, sf::Color(0,0,255));
        }
        auto diagram_centr = generateCentralizedDiagram(mean_points_vec2, AreaBox);
        Vector2<double> centroid_global = {centroid.x + p.x, centroid.y + p.y};
        this->app_gui->drawDiagram(diagram_centr);
        this->app_gui->drawPoint(centroid_global, sf::Color(0,255,255));
        this->app_gui->display();
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - timerstart).count();
    std::cout<<"Computation time cost: -----------------: "<<duration<<" ms\n";

    if (SAVE_CPU_TIME)
    {
        // std::cout << "Sto scrivendo su file\n";
        std::string txt = std::to_string(duration) + "\n";
        this->write_log_file(txt);
    }

}

Eigen::VectorXd Controller::boundVel(Eigen::VectorXd u)
{
    if (u(0) > MAX_LIN_VEL)
    {
        u(0) = MAX_LIN_VEL;
    } else if (u(0) < -MAX_LIN_VEL)
    {
        u(0) = -MAX_LIN_VEL;
    }

    if (u(1) > MAX_LIN_VEL)
    {
        u(1) = MAX_LIN_VEL;
    } else if (u(1) < -MAX_LIN_VEL)
    {
        u(1) = -MAX_LIN_VEL;
    }

    if (u(2) > MAX_ANG_VEL)
    {
        u(2) = MAX_ANG_VEL;
    } else if (u(2) < -MAX_ANG_VEL)
    {
        u(2) = -MAX_ANG_VEL;
    }

    return u;
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

    ros::init(argc, argv, "torch_coverage");
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
