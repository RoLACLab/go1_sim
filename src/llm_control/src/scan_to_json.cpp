#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdlib>

class ScanToJson
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    std::string filepath_;

public:
    ScanToJson()
    {
        ROS_INFO("Node initialized successfully");
        
        // Generate file path in home directory
        char* home = std::getenv("HOME");
        if (home != nullptr) {
            filepath_ = std::string(home) + "/scan_data.json";
        } else {
            filepath_ = "./scan_data.json";  // fallback to current directory
        }
        
        ROS_INFO("Logging scans to: %s", filepath_.c_str());
        
        // Subscribe to laser scan topic
        scan_sub_ = nh_.subscribe("/scan", 1, &ScanToJson::scanCallback, this);
    }
    
    ~ScanToJson()
    {
        ROS_INFO("Shutting down scan logger");
    }

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        try {
            // Create JSON string
            std::ostringstream json_stream;
            json_stream << "{\n";
            
            // Header
            json_stream << "  \"header\": {\n";
            json_stream << "    \"seq\": " << msg->header.seq << ",\n";
            json_stream << "    \"stamp\": {\n";
            json_stream << "      \"secs\": " << msg->header.stamp.sec << ",\n";
            json_stream << "      \"nsecs\": " << msg->header.stamp.nsec << "\n";
            json_stream << "    },\n";
            json_stream << "    \"frame_id\": \"" << msg->header.frame_id << "\"\n";
            json_stream << "  },\n";
            
            // Scan parameters
            json_stream << "  \"angle_min\": " << msg->angle_min << ",\n";
            json_stream << "  \"angle_max\": " << msg->angle_max << ",\n";
            json_stream << "  \"angle_increment\": " << msg->angle_increment << ",\n";
            json_stream << "  \"time_increment\": " << msg->time_increment << ",\n";
            json_stream << "  \"scan_time\": " << msg->scan_time << ",\n";
            json_stream << "  \"range_min\": " << msg->range_min << ",\n";
            json_stream << "  \"range_max\": " << msg->range_max << ",\n";
            
            // Ranges array
            json_stream << "  \"ranges\": [";
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                if (i > 0) json_stream << ", ";
                json_stream << msg->ranges[i];
            }
            json_stream << "],\n";
            
            // Intensities array
            json_stream << "  \"intensities\": [";
            for (size_t i = 0; i < msg->intensities.size(); ++i) {
                if (i > 0) json_stream << ", ";
                json_stream << msg->intensities[i];
            }
            json_stream << "]\n";
            
            json_stream << "}";
            
            // Write to file (overwrites existing content)
            std::ofstream file(filepath_.c_str());
            if (file.is_open()) {
                file << json_stream.str();
                file.close();
                
                // Periodically log
                if (msg->header.seq % 100 == 0) {
                    ROS_INFO("Overwrote file with scan #%u", msg->header.seq);
                }
            } else {
                ROS_ERROR("Unable to open file: %s", filepath_.c_str());
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("Error processing scan: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_to_json");
    
    try {
        ScanToJson logger;
        ROS_INFO("Scan to JSON logger started. Press Ctrl+C to exit.");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Node failed: %s", e.what());
        return 1;
    }
    
    return 0;
}