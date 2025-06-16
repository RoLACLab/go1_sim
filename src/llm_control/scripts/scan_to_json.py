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
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = "scan_data_{}.json".format(timestamp)
        self.filepath = os.path.join(os.path.expanduser("~"), self.filename)
        
        # Initialize JSON Lines file
        self.json_file = open(self.filepath, 'w')
        rospy.loginfo("Logging scans to: %s", self.filepath)
        
        # Subscribe to laser scan topic
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.on_shutdown(self.shutdown_handler)

    def scan_callback(self, msg):
        try:
            # Convert LaserScan message to JSON-serializable dictionary
            scan_dict = {
                "NEW DATA"
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
            
            # Write as JSON Lines format
            self.json_file.write(json.dumps(scan_dict) + '\n')
            
            # Periodically flush to ensure data is written
            if msg.header.seq % 100 == 0:
                self.json_file.flush()
                rospy.loginfo("Logged scan #%d to file", msg.header.seq)
                
        except Exception as e:
            rospy.logerr("Error processing scan: %s", str(e))

    def shutdown_handler(self):
        rospy.loginfo("Shutting down. Closing JSON file.")
        self.json_file.close()

if __name__ == '__main__':
    logger = ScanToJson()
    rospy.loginfo("Scan to JSON logger started. Press Ctrl+C to exit.")
    rospy.spin()