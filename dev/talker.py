#!/usr/bin/env python
import rospy # for writing a ROS node
from std_msgs.msg import String # so we can reuse the std_msgs/String message type (a simple string container) for publishing

def talker():
  pub = rospy.Publisher('chatter', String, queue_size=10) # the node publishes to the chatter topic using the message type String. 
  rospy.init_node('talker', anonymous=True) # name of the node is talker
                                            # anonymous makes the name unique by adding random numbers to the end of the name
  
  rate = rospy.Rate(10) # 10hz is the speed of the data publishing to the topic
                        # "we should expect to go through the loop 10 times per second (as long as our processing time does 
                        # not exceed 1/10th of a second"
  while not rospy.is_shutdown():
    # movement logics probably
    