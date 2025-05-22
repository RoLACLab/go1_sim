#include <ros/ros.h>
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ln");
    ros::NodeHandle nh;
    ROS_INFO("Test");
    ros::Subscriber sub = nh.subscribe("/scan", 1000, chatterCallback);
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    
    ros::spin();
    
    return 0;
}

