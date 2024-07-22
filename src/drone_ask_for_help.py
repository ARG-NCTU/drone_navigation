#! /usr/bin/env python
import numpy as np
import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from behavior_tree_msgs.msg import Active, Status
from geometry_msgs.msg import PoseStamped

FAIL = 0
RUNNING = 1
SUCCESS = 2

class Ask_for_help:
    def __init__(self):
        node_name = "ask_for_human_help"
        self.behavior_active_sub = rospy.Subscriber(
            "/{}_active".format(node_name), Active, self.behavior_active_callback
        )
        self.behavior_status_pub = rospy.Publisher("/{}_status".format(node_name), Status, queue_size=1)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.active = Active()

        self.help_warning_signal_pub = rospy.Publisher("/help_warning_signal", Bool, queue_size=1)
        self.help_signal = Bool()

    def behavior_active_callback(self, msg):
        self.active = msg
    
    def timer_callback(self, event):
        if self.active.active:
            status = Status()
            status.id = self.active.id
            status.status = RUNNING
            self.behavior_status_pub.publish(status)
            self.help_signal.data = True
            self.help_warning_signal_pub.publish(self.help_signal)
        else:
            status = Status()
            status.id = self.active.id
            status.status = FAIL
            self.behavior_status_pub.publish(status)
            self.help_signal.data = False
            self.help_warning_signal_pub.publish(self.help_signal)

if __name__ == "__main__":
    rospy.init_node("ask_for_help")
    human_help = Ask_for_help()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")