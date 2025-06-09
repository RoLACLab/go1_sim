#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan modified_scan = *msg;
  ROS_INFO("[ScanCorruptor] Received scan: Ranges size=%lu, Angle min=%.3f, max=%.3f", 
          modified_scan.ranges.size(),
          modified_scan.angle_min,
          modified_scan.angle_max);
  pub.publish(modified_scan);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "no_mod");
  ros::NodeHandle nh;
  
  // Initialize publisher with the correct message type
  pub = nh.advertise<sensor_msgs::LaserScan>("/scan_corrupt", 1000);
  ros::Subscriber sub = nh.subscribe("/scan", 1000, scanCallback);
  
  ros::spin();
  return 0;
}

