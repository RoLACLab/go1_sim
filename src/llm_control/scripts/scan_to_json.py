#!/usr/bin/env python
import rospy
import json
from sensor_msgs.msg import LaserScan
from datetime import datetime
import os

class ScanToJson:
    def __init__(self):
        rospy.init_node('scan_to_json')
        
        # Generate timestamped filename
        #timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = "scan_data.json" #.format(timestamp)
        self.filepath = os.path.join(os.path.expanduser("~"), self.filename)
        
        rospy.loginfo("Logging scans to: %s", self.filepath)
        
        # Subscribe to laser scan topic
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.on_shutdown(self.shutdown_handler)

        # Initialize file handle (will be opened in write mode later)
        self.json_file = None

    def scan_callback(self, msg):
        try:
            # Convert LaserScan message to JSON-serializable dictionary
            scan_dict = {
                "header": {
                    "seq": msg.header.seq,
                    "stamp": {
                        "secs": msg.header.stamp.secs,
                        "nsecs": msg.header.stamp.nsecs
                    },
                    "frame_id": msg.header.frame_id
                },
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
                "angle_increment": msg.angle_increment,
                "time_increment": msg.time_increment,
                "scan_time": msg.scan_time,
                "range_min": msg.range_min,
                "range_max": msg.range_max,
                "ranges": list(msg.ranges),
                "intensities": list(msg.intensities)
            }
            
            # Open file in write mode (overwrites existing content)
            with open(self.filepath, 'w') as self.json_file:
                # Write entire JSON dictionary to file (single object)
                json.dump(scan_dict, self.json_file)
            
            # Periodically log
            if msg.header.seq % 100 == 0:
                rospy.loginfo("Overwrote file with scan #%d", msg.header.seq)
                
        except Exception as e:
            rospy.logerr("Error processing scan: %s", str(e))

    def shutdown_handler(self):
        rospy.loginfo("Shutting down scan logger")
        # File is already closed by 'with' block, no action needed

if __name__ == '__main__':
    logger = ScanToJson()
    rospy.loginfo("Scan to JSON logger started. Press Ctrl+C to exit.")
    rospy.spin()