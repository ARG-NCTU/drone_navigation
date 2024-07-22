#!/usr/bin/env python

import numpy as np
import rospy
from behavior_tree_msgs.msg import Active, Status
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from drone_navigation.msg import droneWaypoint
import math
import tf
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices, quaternion_from_euler

FAIL = 0
RUNNING = 1
SUCCESS = 2


class DroneReturn:
    # only work in with localization of gazebo mode
    def __init__(self):
        node_name = "return"
        self.return_finish_success_pub = rospy.Publisher("/{}_finish_success".format(node_name), Bool, queue_size=1)
        self.behavior_active_sub = rospy.Subscriber(
            "/{}_active".format(node_name), Active, self.behavior_active_callback
        )
        self.behavior_status_pub = rospy.Publisher("/{}_status".format(node_name), Status, queue_size=1)

        self.drone_pose_to_local_sub = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, self.drone_pose_to_local_callback
        )
        self.drone_takeoff_pose_sub = rospy.Subscriber(
            "/drone/armed_pose", PoseStamped, self.drone_takeoff_pose_callback
        )

        self.waypoint_pub = rospy.Publisher(
            "waypoint_planner/drone_waypoint", droneWaypoint, queue_size=10
        )

        self.waypoint_isfinish_sub = rospy.Subscriber(
            "navigation_manager/is_finish", Bool, self.waypoint_isfinish_callback, queue_size=1
        )

        self.timer = rospy.Timer(rospy.Duration(0.1), self.pub_subgoal_callback)

        self.listener = tf.TransformListener()
        # Get the source frame from the parameter server
        self.source_frame = rospy.get_param('~source_frame', 'map')

        self.active = Active()
        self.return_finish_success = Bool()

        self.subgoal_list = []
        self.planner_name = node_name

        self.first_time_active = True

        self.drone_pose_to_local_received = False
        self.drone_pose_to_local = None
        self.takeoff_pose_to_local_received = False
        self.takeoff_pose_to_local = None
        self.waypoint_isfinish = Bool(data=False)

        self.distance_margin = 0.08
        self.heading_margin = 0.01
        self.origin = "drone"

    def drone_pose_to_local_callback(self, msg):
        self.drone_pose_to_local = msg
        self.drone_pose_to_local_received = True

    def drone_takeoff_pose_callback(self, msg):
        self.takeoff_pose_to_local = msg
        self.takeoff_pose_to_local_received = True

    def behavior_active_callback(self, msg):
        self.active = msg
        if self.active.active and self.first_time_active: # first time active
            self.cache_transforms()
            self.add_return_subgoals()
            self.first_time_active = False

    def waypoint_isfinish_callback(self, msg):
        self.waypoint_isfinish = msg

    def pub_subgoal_callback(self, event):
        if not self.drone_pose_to_local_received or not self.takeoff_pose_to_local_received:
            rospy.logwarn_throttle(1, "Waiting for drone or takeoff pose to be received")
            return
        if self.drone_pose_to_local is None or self.takeoff_pose_to_local is None:
            rospy.logwarn_throttle(1, "Drone or takeoff pose is None")
            return

        if self.active.active:
            if self.subgoal_list != None and len(self.subgoal_list) > 0:
                status = Status()
                status.id = self.active.id
                status.status = RUNNING
                self.behavior_status_pub.publish(status)
                if self.waypoint_isfinish.data:
                    self.subgoal_list.pop(0) #remove the reached subgoal
                    print(self.subgoal_list)
                    self.waypoint_isfinish.data = False
                else:
                    self.waypoint_pub.publish(self.subgoal_list[0])
            else: #subgoal_list is empty
                self.return_finish_success.data = True
                self.return_finish_success_pub.publish(self.return_finish_success)

                status = Status()
                status.id = self.active.id
                status.status = SUCCESS
                self.behavior_status_pub.publish(status)

        else:
            status = Status()
            status.id = self.active.id
            status.status = FAIL
            self.behavior_status_pub.publish(status)

    def add_return_subgoals(self):

        # Step 1: Move up 5 meters
        self.transPose(0, 0, 5.0, 0, True, False)

        # Step 2: Turn to face takeoff position
        dx, dy, dz = self.transform_takeoff_to_drone_pose(self.transform_takeoff_to_drone, 'drone/base_link', self.takeoff_pose_to_local)
        heading = np.arctan2(dy, dx) / math.pi * 180
        print("dx %f, dy %f, dz %f, heading %f", dx, dy, dz, heading)

        self.transPose(0, 0, 5.0, heading, False, False)

        # Step 3: Move to takeoff position
        self.transPose(dx, dy, 5.0, heading, True, False)

        # Step 4: Descend to takeoff platform
        self.transPose(dx, dy, dz, heading, True, True)
    
    def transPose(self, x,y,z, heading, pose_enable, last_waypoint):
        subgoal = droneWaypoint()
        # set position
        subgoal.pose.position.x = x
        subgoal.pose.position.y = y
        subgoal.pose.position.z = z
        #set orientation from heading yaw angle
        yaw = heading * math.pi / 180
        q_new = quaternion_from_euler(0, 0, yaw)
        subgoal.pose.orientation.x = q_new[0]
        subgoal.pose.orientation.y = q_new[1]
        subgoal.pose.orientation.z = q_new[2]
        subgoal.pose.orientation.w = q_new[3]
        subgoal.distance_margin = self.distance_margin
        subgoal.heading_margin = self.heading_margin
        subgoal.origin = self.origin
        subgoal.planner_seq = 0
        subgoal.planner_name = self.planner_name

        subgoal.pose_enable = pose_enable
        subgoal.rotate_enable = True
        subgoal.heading_enable = False
        subgoal.last_waypoint = last_waypoint

        self.subgoal_list.append(subgoal)

    def cache_transforms(self):
        """
        Cache the static transforms from the source frame to the target frames.
        """
        try:
            # Wait for the transformations
            self.listener.waitForTransform('drone/base_link', self.source_frame, rospy.Time(), rospy.Duration(4.0))
            # Get the transformations
            (trans_drone, rot_drone) = self.listener.lookupTransform('drone/base_link', self.source_frame, rospy.Time(0))
            # Convert to transformation matrices
            self.transform_takeoff_to_drone = concatenate_matrices(translation_matrix(trans_drone), quaternion_matrix(rot_drone))

            rospy.loginfo("Transforms cached successfully")
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Error caching transforms: %s", e)
    
    def transform_takeoff_to_drone_pose(self, transform_matrix, target_frame, takeoff_pose):  
        try:
            # Create the pose in the target frame
            takeoff_pose = np.array([takeoff_pose.pose.position.x, takeoff_pose.pose.position.y, takeoff_pose.pose.position.z, 1.0])
            target_pose = np.dot(transform_matrix, takeoff_pose)
            return (target_pose[0], target_pose[1], target_pose[2])
        except Exception as e:
            rospy.logerr("Error in transform_pose: %s", e)

if __name__ == "__main__":
    rospy.init_node("drone_return")
    drone_return = DroneReturn()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
