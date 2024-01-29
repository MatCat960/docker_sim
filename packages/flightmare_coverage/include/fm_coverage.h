#ifndef FM_COVERAGE_H
#define FM_COVERAGE_H

/**
 * @file fm_coverage.h
 * @author {MehdiBelal} ({Mehdi.Belal@tii.ae})
 * @brief TOOD
 * @date 2020-01-31
 */

// STL includes
#include <thread>
#include <random>

// ROS includes
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <visualization_msgs/Marker.h>

// ROS message types includes
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <gazebo_msgs/SetModelState.h>

// boost includes
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// Nodelet plugin generator
#include <pluginlib/class_list_macros.h>

// ARRC library include
#include <arrc/core/state_position.h>
#include <arrc/coverage/voronoi_fortune_coverage.h>

//#define GRAPHICAL_OUTPUT

//#ifdef GRAPHICAL_OUTPUT
#include <grahpics_manager.hpp>
//#endif


template <typename ...Args>
static void empty(Args ...var) { }

#define DEBUG_LEVEL 3
#if DEBUG_LEVEL >=4
#define NODE_VERBOSE(...) ROS_INFO(__VA_ARGS__)
#define NODE_VERBOSE_ONCE(...) ROS_INFO_ONCE(__VA_ARGS__)
#define NODE_VERBOSE_THROTTLE(...) ROS_INFO_THROTTLE(__VA_ARGS__)
#else
#define NODE_VERBOSE(...) empty(__VA_ARGS__)
#define NODE_VERBOSE_ONCE(...) empty(__VA_ARGS__)
#define NODE_VERBOSE_THROTTLE(...) empty(__VA_ARGS__)
#endif

#if DEBUG_LEVEL >=3
#define NODE_INFO(...) ROS_INFO(__VA_ARGS__)
#define NODE_INFO_ONCE(...) ROS_INFO_ONCE(__VA_ARGS__)
#define NODE_INFO_THROTTLE(...) ROS_INFO_THROTTLE(__VA_ARGS__)
#else
#define NODE_INFO(...) empty(__VA_ARGS__)
#define NODE_INFO_ONCE(...) empty(__VA_ARGS__)
#define NODE_INFO_THROTTLE(...) empty(__VA_ARGS__)
#endif

#if DEBUG_LEVEL >=2
#define NODE_WARN(...) ROS_WARN(__VA_ARGS__)
#define NODE_WARN_ONCE(...) ROS_WARN_ONCE(__VA_ARGS__)
#else
#define NODE_WARN(...) empty(__VA_ARGS__)
#define NODE_WARN_ONCE(...) empty(__VA_ARGS__)
#endif

#if DEBUG_LEVEL >=1
#define NODE_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define NODE_ERROR_ONCE(...) ROS_ERROR_ONCE(__VA_ARGS__)
#else
#define NODE_ERROR(...) empty(__VA_ARGS__)
#define NODE_ERROR_ONCE(...) empty(__VA_ARGS__)
#endif

namespace swarmros
{

/**
 * @brief The DroneState enum, minimal raprpesentation of
 *      a state machine
 */
enum class DroneState
{
    DRONE_STATE_WAIT_AND_ARM = 0,
    DRONE_STATE_START,
    DRONE_STATE_ONGOING,
    DRONE_STATE_COMPLETED
};

static DroneState& operator++(DroneState& e, int)
{
    if(e == DroneState::DRONE_STATE_ONGOING) { }
    else
    {
        e = static_cast<DroneState>(static_cast<int>(e) + 1);
    }
    return e;
}

/**
 * @brief The Environment struct, structure used to rappresent a portion of the space
 *          squared descibed by the position of the left corner, and the sizes along
 *          x and y axes
 */
struct Environment
{
    double area_l;
    double area_b;
    double area_sx;
    double area_sy;

    Environment() { }
};


/**
 * @brief Flightmare_Coverage : nodelet implementing the controller for the Coverage flightmare porject
 *
 * This project contains the tools to run a Flightmare-based simulation of a coverage use case.
 * The logic and algorithm implementation are part of the ARRC library. The simulation environment is instantiated through a docker image.
 *
 * The scope of this nodelet is to manage the entire simulation and to require the different coverage call steps for each drone.
 * Actually, even if the structure of the system is not really to be considered distributed (given that the calculation of each
 * step of the diagram is carried out in a centralized way by a single node, taking into account all the measurements at each step),
 * the structure of the algorithm still allows you to make a calculation of the diagram for each drone, at each iteration, making
 * the execution centralized only in the calculation but not in the logic.
 *
 * @note The ROS/params folder contains a series of settings for selecting the environment. There is a file for managing the
 * environment in the case of geometric distribution and one in case of Gaussian distribution, the selection of one case of
 * another must be set through the default.yaml file using the gaussian_distribution parameter [true/false], by default it
 * is set to the Gaussian.
 *
 * @note Currently it is necessary to manually synchronize the number of drones that are selected in the launch file
 * to those that are defined in the default.yaml. This mechanism is not yet managed automatically so if the two elements
 * are not consistent with each other, runtime errors may occur since the number of drones it has defined in the
 * configuration file is not equal to that of the simulation environment.
 *
 */
class Flightmare_Coverage : public nodelet::Nodelet
{
public:

    virtual void onInit();

private:
    // Coverage algorithm
    arrc::algorithms::VoronoiFortuneCoverage<double> coverage;

    // ROS nodelet
    ros::NodeHandle nh;

    // ROS parameters
    int scende_id = 0;
    float main_loop_freq = 15;
    int number_of_drones = 16;
    float drone_wait_flight_time = 0;
    bool unity_render = false;
    bool _gui_ = true;
    bool gaussian_distribution = true;
    std::string configuration_path;
    double pose_noise_stddev;

    // Drone management
    DroneState current_state;

    // ROS publishers for drones management
    std::vector<ros::Publisher> publishersArm;
    std::vector<ros::Publisher> publishersStart;
    std::vector<ros::Publisher> publishersOff;
    std::vector<ros::Publisher> publishersPose;
    std::vector<ros::Publisher> publishersVelocity;
    std::vector<ros::Publisher> publishersPolygon;
    std::vector<ros::Publisher> publishersCentroid;
    ros::Publisher visual_topic_gaussian;
    ros::Publisher visual_topic_area;

    ros::ServiceClient service_model;

    // ROS subscribers
    ros::Subscriber sub_pose;
    std::vector<ros::Subscriber> subStateEstimator;

    // ROS timers
    ros::Timer timer_main, timer_variant_shape, timer_simulation;

    // Current position of the different drones
    Eigen::VectorXd pose_x;
    Eigen::VectorXd pose_y;

    Eigen::VectorXd pose_x_csv;
    Eigen::VectorXd pose_y_csv;

    // Random noise generation
    std::default_random_engine generator;
    
    // Envoirement managenent
    uint counter = 0;
    uint last_counter_hit = 0;
    std::atomic<bool> keep_runing = { false };
    std::mutex m;
    double angle = 0;
    std::vector<Gaussian<double>> gaussians;
    std::string global_file_name;

    // list of parameters for simulazione
    std::vector<double> area_size{50.0, 100.0};
    std::vector<double> robot_range{10, 50, 100, 500};

    bool sim_type = false; //  false = no noise, initial value
    int current_area_size_index = 0;
    int current_robot_range_index = 0;


    double area_size_value;
    double robot_range_value;
    int iteration_value = 0;
    std::string simulation_type;

    std::ofstream myfile;
    bool record_csv = false;

#ifdef GRAPHICAL_OUTPUT
    GraphicsManager<double>* graphics_manager = nullptr;
#endif

    /**
     * @brief poseCallback - method to retrive the pose of the different drones, the id of the item
     *          is managed via the unsigned index variable
     *
     * @param msg - nav_msgs::Odometry, position of the drone
     * @param index - index of the drone
     */
    void poseCallback(const nav_msgs::Odometry::ConstPtr &msg, unsigned index);

    /**
     * @brief mainTask - method managed by the timer in order to periodally manage the drone envoirement,
     *          basing on the current conditions of the test
     *
     * @param event - ros::TimerEvent NOT USED
     */
    void mainTask(const ros::TimerEvent &event);

    void simulation_task(const ros::TimerEvent &event);

    /**
     * @brief readArea - method used to retrive the parameters that describe the coverage area of this experiment,
     *          the results fo this method is used to visualize in RVIZ the coverage area for debug porpuses
     *
     * @param config_file - std::string, file name from which retrive the parameters
     *
     * @return res - Environment, value of the 4 parameters
     */
    Environment read_area(std::string config_file);

    /**
     * @brief read_gaussian_parameters - method used to read the parameters that rappresents the gaussian potentials
     *                                   for the Voronoi tassselation. it is invoked only when the disitribution of
     *                                   the the space belogs to this type
     *
     * @param config_file - std::string, file name from which retrive the parameters
     *
     * @return std::vector<Gaussian<double>>, list of all the items in the config file
     */
    std::vector<Gaussian<double> > read_gaussian_parameters(std::string config_file);

    /**
     * @brief drawEnvoirement - utility method used to visualize the envoirement area used for the coverage
     */
    void draw_environment(std::string file_name);

};

}

#endif // FM_COVERAGE_H
