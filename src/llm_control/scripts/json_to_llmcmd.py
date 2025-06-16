#!/usr/bin/env python
import rospy
import json
import sys
from std_msgs.msg import String

class LLMCommandPublisher:
    def __init__(self, json_file_path):
        rospy.init_node('llm_command_publisher')
        self.pub = rospy.Publisher('/llm_cmd', String, queue_size=10, latch=True)
        rospy.loginfo("LLM Command Publisher started. Reading from: %s", json_file_path)
        
        try:
            # Read and parse JSON file
            with open(json_file_path, 'r') as f:
                action_plan = json.load(f)
            
            # Convert to JSON string and publish
            json_str = json.dumps(action_plan)
            self.pub.publish(json_str)
            rospy.loginfo("Published LLM command: %s", json_str)
            
        except Exception as e:
            rospy.logerr("Error processing JSON file: %s", str(e))

    def keep_alive(self):
        """Keep the node running to maintain the latched message"""
        rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        rospy.logerr("Usage: rosrun llm_control json_to_llm_cmd.py <path_to_json_file>")
        sys.exit(1)
        
    publisher = LLMCommandPublisher(sys.argv[1])
    publisher.keep_alive()