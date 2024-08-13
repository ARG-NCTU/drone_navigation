#!/usr/bin/env python
import rospy
import math
import yaml
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from behavior_tree_msgs.msg import Active

class DroneWaypointUnity:
    def __init__(self):
        self.node_name = rospy.get_param("~node_name", "return")
        self.waypoint_file_path = rospy.get_param("~waypoint_file_path", "waypoints.yaml")
        
        self.behavior_active_sub = rospy.Subscriber(
            "/{}_active".format(self.node_name), Active, self.behavior_active_callback
        )
        
        # Publisher for waypoints as PoseArray
        self.waypoint_array_pub = rospy.Publisher(
            "waypoint_planner/waypoints_array", PoseArray, queue_size=10
        )
        
        self.drone_pose_to_local_sub = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, self.drone_pose_to_local_callback
        )
        
        self.subgoal_list = []
        self.first_time_active = True
        self.drone_pose = None
        self.transform_position_list = []
        self.status = None
        self.pose_array = None

        # Load waypoints from YAML
        self.load_waypoints()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_waypoints)

    def behavior_active_callback(self, msg):
        if msg.active:
            self.status = msg.active
            if self.first_time_active:
                self.first_time_active = False
                self.transform_subgoal()
                self.append_waypoints()
        else:
            self.status = msg.active
            self.first_time_active = True

    def drone_pose_to_local_callback(self, msg):
        self.drone_pose = msg.pose
    
    def transform_subgoal(self):
        if self.drone_pose is None:
            rospy.logwarn("Drone pose not received yet")
            return
        
        drone_origin_pose = [self.drone_pose.position.x, self.drone_pose.position.y, self.drone_pose.position.z]
        origin_y = self.get_yaw_from_quaternion(self.drone_pose.orientation)

        for waypoint in self.subgoal_list:
            transformed_position = [0, 0, 0]

            if waypoint[0] == 0:
                if waypoint[1] >= 0:
                    transformed_position[0] = drone_origin_pose[0] + \
                        math.sqrt(pow(waypoint[1], 2)) * math.cos(origin_y + math.pi / 2)
                    transformed_position[1] = drone_origin_pose[1] + \
                        math.sqrt(pow(waypoint[1], 2)) * math.sin(origin_y + math.pi / 2)
                else:
                    transformed_position[0] = drone_origin_pose[0] + \
                        math.sqrt(pow(waypoint[1], 2)) * math.cos(origin_y - math.pi / 2)
                    transformed_position[1] = drone_origin_pose[1] + \
                        math.sqrt(pow(waypoint[1], 2)) * math.sin(origin_y - math.pi / 2)
            else:
                transformed_position[0] = drone_origin_pose[0] + \
                    math.sqrt(pow(waypoint[0], 2) + pow(waypoint[1], 2)) * \
                    math.cos(origin_y + math.atan2(waypoint[1], waypoint[0]))
                transformed_position[1] = drone_origin_pose[1] + \
                    math.sqrt(pow(waypoint[0], 2) + pow(waypoint[1], 2)) * \
                    math.sin(origin_y + math.atan2(waypoint[1], waypoint[0]))

            transformed_position[2] = drone_origin_pose[2] + waypoint[2]
            self.transform_position_list.append(transformed_position)
            

    def get_yaw_from_quaternion(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def load_waypoints(self):
        try:
            with open(self.waypoint_file_path, 'r') as file:
                yaml_data = yaml.safe_load(file)
                
            for wp in yaml_data['waypoint']:
                waypoint = wp
                self.subgoal_list.append(waypoint)
                
            rospy.loginfo("Waypoints loaded successfully from {}".format(self.waypoint_file_path))
        except Exception as e:
            rospy.logerr("Failed to load waypoints: {}".format(e))

    def append_waypoints(self):
        self.pose_array = PoseArray()
        for waypoint in self.transform_position_list:
            pose = Pose()
            pose.position.x = waypoint[0]
            pose.position.y = waypoint[1]
            pose.position.z = waypoint[2]
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            self.pose_array.poses.append(pose)
        
    def publish_waypoints(self, event):
        if self.pose_array is not None and self.status:
            self.waypoint_array_pub.publish(self.pose_array)

if __name__ == "__main__":
    rospy.init_node("drone_waypoint")
    drone_waypoint = DroneWaypointUnity()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
