#include "fm_coverage.h"

#include <boost/foreach.hpp>
#include <iostream>       // std::cout, std::endl
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include <dirent.h>
#include <errno.h>
#include <boost/filesystem.hpp>

namespace swarmros
{

template <typename T>
bool contains(std::vector<Vector2<T> > list, Vector2<T> item)
{
    bool res = false;
    for(Vector2<T> _l : list)
    {
        if( (_l - item).getNorm() < 0.5)
        {
            res = true;
            break;
        }
    }

    return res;
}


void Flightmare_Coverage::onInit()
{

    ROS_INFO("Initializing ROS callback queue ..");
    // ROS Nodelet callback queue
    nh = nodelet::Nodelet::getMTPrivateNodeHandle();
    ros::Time::waitForValid();

    // Params stored in default.yaml
    nh.getParam("scende_id", scende_id);
    nh.getParam("main_loop_freq", main_loop_freq);
    nh.getParam("number_of_drones", number_of_drones);
    nh.getParam("drone_wait_flight_time", drone_wait_flight_time);
    nh.getParam("unity_render", unity_render);
    nh.getParam("gui", _gui_);
    nh.getParam("gaussian_distribution", gaussian_distribution);
    nh.getParam("configuration_path", configuration_path);
    nh.getParam("pose_noise_stddev", pose_noise_stddev);

    nh.getParam("area_size", area_size_value);
    nh.getParam("robot_range", robot_range_value);
    nh.getParam("iteration", iteration_value);
    nh.getParam("type", simulation_type);


    // Subcribers and publishers
    for (int i = 1; i <= number_of_drones; i++)
    {
        // character to replace to generate the subscriber name
        std::string pubArmTemplate = "/hummingbird" + std::to_string(i) + "/bridge/arm";
        std::string pubStartTemplate = "/hummingbird" + std::to_string(i) + "/autopilot/start";
        std::string pubOffTemplate = "/hummingbird"+ std::to_string(i) + "/autopilot/off";
        std::string pubPoseTemplate = "/hummingbird" + std::to_string(i) + "/autopilot/pose_command";
        std::string pubVelocityTemplate = "/hummingbird" + std::to_string(i) + "/autopilot/velocity_command";
        std::string pubTemplate = "/swarming/state_estimate" + std::to_string(i);
        std::string polyTemplate = "voronoi_cell_"+std::to_string(i);
        std::string centroidTemplate = "voronoi_centroid_"+std::to_string(i);
        // publisher for arming
        publishersArm.push_back(nh.advertise<std_msgs::Bool>(pubArmTemplate, 1));
        // publisher for start
        publishersStart.push_back(nh.advertise<std_msgs::Empty>(pubStartTemplate, 1));
        // Publisher to off the autopilot
        publishersOff.push_back(nh.advertise<std_msgs::Empty>(pubOffTemplate, 1));
        // publisher for pose command
        publishersPose.push_back(nh.advertise<geometry_msgs::PoseStamped>(pubPoseTemplate, 1));
        // publisher for velocity command
        publishersVelocity.push_back(nh.advertise<geometry_msgs::TwistStamped>(pubVelocityTemplate, 1));
        // the subscribers subscribe all to the same callback that then redirect to specific drone using the i argument
        sub_pose = nh.subscribe<nav_msgs::Odometry>(pubTemplate, 1, boost::bind(&Flightmare_Coverage::poseCallback, this, _1, i));
        subStateEstimator.push_back(sub_pose);
        // voronoi cell publishers
        publishersPolygon.push_back(nh.advertise<geometry_msgs::PolygonStamped>(polyTemplate,1));
        // voronoi centroid publishers
        publishersCentroid.push_back(nh.advertise<visualization_msgs::Marker>(centroidTemplate,1));
    }

    visual_topic_gaussian = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0);
    visual_topic_area = nh.advertise<geometry_msgs::PolygonStamped>( "visualization_area", 0);

    service_model = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", 0);

    ROS_INFO("Initializing Coverage algorithm ..");
    // Initialize coverage algorithm
    coverage = arrc::algorithms::VoronoiFortuneCoverage<double>();
    gaussian_distribution = (simulation_type == "gaussian");

    if(!gaussian_distribution)
        global_file_name = configuration_path + "/coverage_geometric.json";
    else
    {
        global_file_name = configuration_path + "/coverage_gaussian.json";
        gaussians = read_gaussian_parameters(global_file_name);
    }

    coverage.loadConfig(global_file_name);

    ROS_INFO("Initializing parameters ..");
    // Initialize global parameters
    pose_x = Eigen::VectorXd::Zero(number_of_drones);
    pose_y = Eigen::VectorXd::Zero(number_of_drones);
    pose_x_csv = Eigen::VectorXd::Zero(number_of_drones);
    pose_y_csv = Eigen::VectorXd::Zero(number_of_drones);

    ROS_INFO("Initializing main thread function ..");
    // Main thread task and state machine
    current_state = DroneState::DRONE_STATE_WAIT_AND_ARM;
    timer_main = nh.createTimer(ros::Duration(0.3), &Flightmare_Coverage::mainTask, this, false, false);
    timer_variant_shape = nh.createTimer(ros::Duration(0.1), [this](const ros::TimerEvent &event) {
        // We consider to move the mean in a circoular shape centered in [5, 5]
        // and with 2 meters radius
        double radius = 3;
        double x = (radius * cos(angle)) + 5;
        double y = (radius * sin(angle)) + 5;
        double var = (cos(angle)) + 2;

        angle += 0.01;
        coverage.setGaussianMean(Vector2<double>(x, y));
        coverage.setGaussianVariance(var);

        gaussians.clear();
        gaussians.push_back(Gaussian(Vector2<double>(x, y), var));

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = var;
        marker.scale.y = var;
        marker.scale.z = 0.01;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        visual_topic_gaussian.publish(marker);
    }, false, false);

    timer_simulation = nh.createTimer(ros::Duration(0.3), &Flightmare_Coverage::simulation_task, this, false, true);

    //Draw environment area for RVIZ visualization
    draw_environment(global_file_name);

#ifdef GRAPHICAL_OUTPUT
    Environment _env = read_area(global_file_name);
    graphics_manager = new GraphicsManager<double>(_env.area_sx, _env.area_sy, _env.area_l, _env.area_b, 0.2);
#endif

}


void Flightmare_Coverage::poseCallback(const nav_msgs::Odometry::ConstPtr &msg, unsigned index)
{
    std::normal_distribution<double> dist(0.0,pose_noise_stddev);

    //NODE_VERBOSE("New value for item %d : [%f, %f, z=don't care]", index, msg->pose.pose.position.x, msg->pose.pose.position.y);
    m.lock();
    pose_x(index - 1) = msg->pose.pose.position.x + dist(generator);
    pose_y(index - 1) = msg->pose.pose.position.y + dist(generator);
    // Write positions for the output .csv file
    pose_x_csv(index - 1) = msg->pose.pose.position.x;
    pose_y_csv(index - 1) = msg->pose.pose.position.y;
    m.unlock();
}


void Flightmare_Coverage::simulation_task(const ros::TimerEvent &event)
{
    NODE_WARN("Running iteration number %d with AREA %f value and ROBOT range %f", iteration_value, area_size_value, robot_range_value);

    // Set the area parameter
    coverage.setEnvoirement(-area_size_value/2, -area_size_value/2, area_size_value, area_size_value);

    // Set the range parameter
    coverage.setRobotRange(robot_range_value);

    // Set drones initial positions
    std::vector<Vector2<double> > _initial_pos;
    for(int i = 0; i < number_of_drones; i++)
    {
        gazebo_msgs::SetModelState state;
        state.request.model_state.model_name = "hummingbird" + std::to_string(i+1);

        std::normal_distribution<double> dist(0.0, 1.0);
        double value = dist(generator);
        std::random_device rd; // obtain a random number from hardware
        std::mt19937 gen(rd()); // seed the generator
        std::uniform_int_distribution<> distr(-4, 4); // define the range
        double val = distr(gen) + dist(generator);
        Vector2<double> v;
        do
        {
            v = Vector2<double>(distr(gen) + dist(generator), distr(gen) + dist(generator));
        } while(contains(_initial_pos, v));

        _initial_pos.push_back(v);

        state.request.model_state.pose.position.x = v.x;
        state.request.model_state.pose.position.y = v.y;
        state.request.model_state.pose.position.z = 0.1;
        state.request.model_state.pose.orientation.w = 1;

        // chiamare serv posizionament
        service_model.call(state);
    }

    // Restore simulation to initial point
    counter = 0;
    current_state = DroneState::DRONE_STATE_WAIT_AND_ARM;


    std::string filename = "record_" + std::to_string(iteration_value) +
                           "_area_" + std::to_string(area_size_value) +
                           "_range_" + std::to_string(robot_range_value) + ".csv";

    std::string path = "/home/tii/workspace/src/flightmare_coverage/results/" +
                        simulation_type +
                        "/drones_" + std::to_string(number_of_drones) +
                        "/area_size" + std::to_string(area_size_value) +
                        "/range_" + std::to_string(robot_range_value);

    DIR* dir = opendir(path.c_str());
    if (dir)
    {
        ROS_ERROR("folder for logs %s exists", path.c_str());
        closedir(dir);
    }
    else
    {
        ROS_ERROR("Creating folder for logs %s", path.c_str());
        boost::filesystem::create_directories(path);

    }
    myfile.open(path + "/" + filename);
    record_csv = true;
    myfile << "timestamp,";
    for(int i = 0; i < number_of_drones; i++)
    {
        myfile << "x" << std::to_string(i) << ",";
        myfile << "y" << std::to_string(i) << ",";
    }
    myfile << "\n";

    // avviare maintask
    timer_main.start();

    // sleep the current thread for N seconds, time of simulation
    int time = 120;
    ROS_WARN("Sleeping for %d second", time);
    //std::this_thread::sleep_for(std::chrono::seconds(time));
    ros::Duration(time).sleep();
    ROS_WARN("END sleeping");
    // Stop the simulation and the recording file
    keep_runing = false;
    timer_main.stop();
    record_csv = false;
    myfile.close();

    NODE_WARN("EXIT PROGRAM");
    //wait 5 seconds before the next iteration
    std::this_thread::sleep_for(std::chrono::seconds(5));
    exit(0);

}


void Flightmare_Coverage::mainTask(const ros::TimerEvent &event)
{
    ROS_INFO_ONCE("[Flightmare_Coverage] starting main task");
    counter++;

    switch (current_state) {
    case DroneState::DRONE_STATE_WAIT_AND_ARM:
    {
        if (counter > 25)
        {
            ROS_INFO("ARMING DRONES ...");
            std_msgs::Bool arm;
            arm.data = true;
            for (auto &pub : publishersArm) { pub.publish(arm); }
            current_state++;

        }
    }
    break;
    case DroneState::DRONE_STATE_START:
    {
        if (counter > 40)
        {
            ROS_INFO("RUNNING DRONES ...");
            std_msgs::Empty start;
            for (auto &pub : publishersStart) { pub.publish(start); }
            keep_runing = true;
            current_state++;
        }
    }
    break;
    case DroneState::DRONE_STATE_ONGOING:
    {
        if (counter > 70)
        {
            //timer_variant_shape.start();

            ROS_INFO("STARTING COVERAGE ...");
            while (keep_runing && ros::ok())
            {
                ROS_INFO_THROTTLE(3.0, "COVERAGE ONGOING ...");
                m.lock();
                //populate seeds with new neightbors positions
                std::vector<Vector2<double>> seeds;
                for (int i = 0; i < number_of_drones; i++)
                {
                    // if ((pose_x(i) != 0.0) && (pose_y(i) != 0.0))
                    // {
                        Vector2<double> _item(pose_x(i), pose_y(i));
                        seeds.push_back(_item);
                        NODE_VERBOSE("Drone number %d is in [%f, %f]", i, pose_x(i), pose_y(i));
                    // }
                    // else
                    // {
                    //     // better manage this situation!
                    //     std::cerr << "item number " << i << " was exluded" << std::endl;
                    // }
                }
                m.unlock();

                //doStep for every robot
                for(int i = 0; i < number_of_drones; i++)
                {
                    NODE_VERBOSE("Starting step for robot %d",i);
                    coverage.doStep(seeds, seeds.at(i));
                    NODE_VERBOSE("Done step for robot %d",i);
                    std::tuple<double, double, double> speed = coverage.getSpeed();
                    geometry_msgs::TwistStamped velocity;
                    velocity.twist.linear.x = std::get<0>(speed);
                    velocity.twist.linear.y = std::get<1>(speed);
                    velocity.twist.linear.z = std::get<2>(speed);
                    NODE_VERBOSE("Got speed for robot %d",i);
                    // NODE_VERBOSE("Speed for item %d: [%f, %f, %f]", i, velocity.twist.linear.x,
                    //                                                    velocity.twist.linear.y,
                    //                                                    velocity.twist.linear.z);
                    // // For every drone then, publish the speed
                    publishersVelocity.at(i).publish(velocity);

                    // publish voronoi cell
                    geometry_msgs::PolygonStamped polygon;
                    polygon.header.frame_id = "world";
                    polygon.header.stamp = ros::Time::now();
                    geometry_msgs::Point32 _p;
                    auto diagram = coverage.getCurrentDiagram();
                    auto halfEdge = diagram.getFace(0)->outerComponent;
                    do
                    {
                        _p.x = halfEdge->origin->point.x+pose_x(i);
                        _p.y = halfEdge->origin->point.y+pose_y(i);
                        _p.z = 0.0;
                        polygon.polygon.points.push_back(_p);
                        halfEdge = halfEdge->next;                                                  //passaggio all'half-edge successivo della face 0
                    } while(halfEdge != diagram.getFace(0)->outerComponent); 
                    publishersPolygon.at(i).publish(polygon);

                    // publish voronoi centroid
                    auto centroid = coverage.getCurrentCentroid();
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "world";
                    marker.header.stamp = ros::Time::now();
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.pose.position.x = centroid.x+pose_x(i);
                    marker.pose.position.y = centroid.y+pose_y(i);
                    marker.pose.position.z = 0.15;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.1;
                    marker.scale.y = 0.1;
                    marker.scale.z = 0.01;
                    marker.color.r = 0.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 1.0f;
                    marker.color.a = 1.0;
                    publishersCentroid.at(i).publish(marker);
                    NODE_VERBOSE("Done publishing for robot %d",i);

                }

                if(myfile.is_open())
                {
                    myfile << ros::Time::now() << ",";
                    for(int i = 0; i < number_of_drones; i++)
                    {
                        myfile << pose_x_csv(i) << ",";
                        myfile << pose_y_csv(i) << ",";
                    }
                    myfile << "\n";

                }


                draw_environment(global_file_name);

                /* This section allows to show the shape of the Voronoi Diagram in order
                    to give visual instruments for the output, in a more real envoirement,
                    where the calculations of the diagrams are distribuited, an entry point
                    for this kind of operations could not be available */
#ifdef GRAPHICAL_OUTPUT
                Environment _env = read_area(global_file_name);
                Box<double> box{_env.area_l, _env.area_b, _env.area_l + _env.area_sy, _env.area_b + _env.area_sy};
                NODE_VERBOSE("generateCentralizedDiagram with %d items", (unsigned) seeds.size());
                auto _diagram = coverage.generateCentralized_Diagram(seeds, box);
                graphics_manager->drawDiagram(_diagram, gaussians);
                //if(gaussian_distribution) { ROS_ERROR("123123"); graphics_manager->drawGaussians(gaussians); }
#endif
            }

        current_state++;
        }
    }
    break;
    default:
        break;
    }
}


Environment Flightmare_Coverage::read_area(std::string config_file)
{
    Environment result;
    try
    {
        using value_type = boost::property_tree::ptree::value_type;
        boost::property_tree::ptree tree;
        boost::property_tree::read_json(config_file, tree);

        //threedimensional = tree.get<bool>("threedimensional"); //TODO: needed for drones?
        result.area_l = tree.get<double>("areaLeft", 0.0);
        result.area_b = tree.get<double>("areaBottom", 0.0);
        result.area_sx = tree.get<double>("areaSizeX", 0.0);
        result.area_sy = tree.get<double>("areaSizeY", 0.0);

        return result;
    }
    catch(std::exception const& e)
    {
        NODE_ERROR("[Flightmare_Coverage] Error on reading configuration file: ", e.what());
        return Environment();
    }
}


std::vector<Gaussian<double>> Flightmare_Coverage::read_gaussian_parameters(std::string config_file)
{
    std::vector<Gaussian<double>>  result;
    try
    {
        using value_type = boost::property_tree::ptree::value_type;
        boost::property_tree::ptree tree;
        boost::property_tree::read_json(config_file, tree);
        boost::property_tree::ptree::const_assoc_iterator it = tree.find("centroids");
        if( it == tree.not_found() )
        {
            throw std::runtime_error("[Flightmare_Coverage] Configuration file does not contain valid gaussian parameters");
        }

        BOOST_FOREACH(value_type const &v, tree.get_child("centroids"))
        {
            const boost::property_tree::ptree &subtree = v.second;
            double _var = subtree.get<double>("var");

            Vector2<double> _mean;
            std::vector<double> _v;
            BOOST_FOREACH(value_type const &k, subtree.get_child("mean")) { _v.push_back(k.second.get_value<double>()); }
            _mean = Vector2<double>::fromstdVector(_v);

            result.push_back(Gaussian(_mean, _var));
        }
    }
    catch(std::exception const& e)
    {
        NODE_ERROR("[Flightmare_Coverage] Error on reading configuration file: ", e.what());
        return result;
    }

    return result;
}


void Flightmare_Coverage::draw_environment(std::string file_name)
{
    Environment _env = read_area(file_name);
    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = "world";
    polygon.header.stamp = ros::Time::now();
    geometry_msgs::Point32 _p;

    _p.x = _env.area_l;
    _p.y = _env.area_b;
    _p.z = 0;
    polygon.polygon.points.push_back(_p);

    _p.x = _env.area_l + _env.area_sx;
    _p.y = _env.area_b;
    _p.z = 0;
    polygon.polygon.points.push_back(_p);

    _p.x = _env.area_l + _env.area_sx;
    _p.y = _env.area_b + _env.area_sy;
    _p.z = 0;
    polygon.polygon.points.push_back(_p);

    _p.x = _env.area_l;
    _p.y = _env.area_b + _env.area_sy;
    _p.z = 0;
    polygon.polygon.points.push_back(_p);

    visual_topic_area.publish(polygon);
}

}

PLUGINLIB_EXPORT_CLASS(swarmros::Flightmare_Coverage, nodelet::Nodelet)

