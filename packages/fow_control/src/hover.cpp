#include <ros/ros.h>
#include <fow_control/FowController.h>
#include <tf2/utils.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

class HoverCommander
{
private:
    ros::Publisher arm_pub;
    ros::Publisher hover_pub;
    

public:
    ros::NodeHandle n;

    HoverCommander()
    {
        arm_pub = n.advertise<std_msgs::Bool>("/bridge/arm", 1);
        hover_pub = n.advertise<std_msgs::Empty>("/autopilot/start", 1);
    }

    void arm()
    {
        std_msgs::Bool msg;
        msg.data = true;
        arm_pub.publish(msg);
    }

    void hover()
    {
        std_msgs::Empty msg;
        hover_pub.publish(msg);
    }


    
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "starter_node");
    HoverCommander Commander;

    // Wait for autopilot startup
    // ros::service::waitForService("/autopilot/get_loggers");
    sleep(10);
    ROS_INFO("Node started. Arming...");
    sleep(1);
    Commander.arm();
    ROS_INFO("Armed. Hovering...");
    sleep(1);
    Commander.hover();
    ROS_INFO("Hovering. Exiting...");
    sleep(1);


    return 0;
}