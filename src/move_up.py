#!/usr/bin/env python
import rospy
from behavior_tree_msgs.msg import Active, Status
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State  # Assuming you're using mavros_msgs for UAV state
from std_msgs.msg import Bool  # Assuming Active message is of type Bool

# Define behavior tree states
FAIL = 0
RUNNING = 1
SUCCESS = 2


class MultiAxisMovement:
    def __init__(self, node_name="move_up"):
        """
        Initialize the MultiAxisMovement class with linear and angular velocities.
        """
        rospy.init_node('multi_axis_movement_node', anonymous=True)
        
        # Subscribe to active behavior state
        self.behavior_active_sub = rospy.Subscriber(
            "/{}_active".format(node_name), Active, self.behavior_active_callback
        )

        # Subscribe to UAV state (e.g., using mavros/state)
        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self.state_callback
        )
        
        # Publish status to /move_up_status topic
        self.status_pub = rospy.Publisher('/{}_status'.format(node_name), Status, queue_size=10)

        # Publish velocity commands to the standard cmd_vel_unstamped topic
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        
        # UAV current state
        self.current_state = None
        self.is_behavior_active = False  # Track whether the behavior is active
        
        # Set linear speed for each axis (meters per second)
        self.x_speed = 0.0  # X-axis speed
        self.y_speed = 0.0  # Y-axis speed
        self.z_speed = 1.0  # Z-axis (vertical) speed
        
        # Set angular speed for each axis (radians per second)
        self.roll_speed = 0.0   # Angular velocity around X-axis (roll)
        self.pitch_speed = 0.0  # Angular velocity around Y-axis (pitch)
        self.yaw_speed = 0.0    # Angular velocity around Z-axis (yaw)
        
        # Set publishing frequency
        self.rate = rospy.Rate(20)  # 20 Hz

    def state_callback(self, state_msg):
        """
        Callback to update the current UAV state.
        
        Parameters:
        state_msg (State): The current UAV state message.
        """
        self.current_state = state_msg
        rospy.loginfo("Current UAV state: Mode - %s, Armed - %s, Connected - %s",
                      self.current_state.mode, self.current_state.armed, self.current_state.connected)

    def behavior_active_callback(self, msg):
        """
        Callback to update the active behavior state.
        
        Parameters:
        msg (Active): The message indicating whether the behavior is active.
        """
        self.is_behavior_active = msg.active
        self.id = msg.id

    def move_in_three_axes(self):
        """
        Continuously move the UAV in all three axes (X, Y, Z) and apply angular velocities (roll, pitch, yaw),
        but only if the behavior is active.
        """
        # Ensure UAV is connected and mode is set correctly
        while not rospy.is_shutdown() and self.current_state is None:
            rospy.sleep(0.1)

        if not self.current_state or not self.current_state.connected:
            rospy.logwarn("UAV is not connected.")
            return

        # Construct velocity command for linear and angular movement
        twist = Twist()
        twist.linear.x = self.x_speed  # X-axis movement
        twist.linear.y = self.y_speed  # Y-axis movement
        twist.linear.z = self.z_speed  # Z-axis (vertical) movement
        twist.angular.x = self.roll_speed   # Roll (around X-axis)
        twist.angular.y = self.pitch_speed  # Pitch (around Y-axis)
        twist.angular.z = self.yaw_speed    # Yaw (around Z-axis)

        rospy.loginfo("Waiting for behavior to become active...")

        # Continuously publish velocity command and status only when behavior is active
        while not rospy.is_shutdown():
            if self.is_behavior_active:
                self.velocity_pub.publish(twist)
                rospy.loginfo("Publishing speeds - Linear (X: %.2f, Y: %.2f, Z: %.2f) | Angular (Roll: %.2f, Pitch: %.2f, Yaw: %.2f)",
                              self.x_speed, self.y_speed, self.z_speed, self.roll_speed, self.pitch_speed, self.yaw_speed)

                # Log current UAV state
                if self.current_state:
                    rospy.loginfo("Current UAV state: Mode - %s, Armed - %s, Connected - %s",
                                  self.current_state.mode, self.current_state.armed, self.current_state.connected)

            # Create and publish status message
            status_msg = Status()
            status_msg.id = self.id  # Assuming Active msg has an id field
            status_msg.status = RUNNING
            self.status_pub.publish(status_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # You can specify the node name here
        multi_axis_movement = MultiAxisMovement(node_name="move_up")
        multi_axis_movement.move_in_three_axes()
    except rospy.ROSInterruptException:
        pass
