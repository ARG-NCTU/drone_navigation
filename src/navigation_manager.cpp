#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <drone_navigation/droneWaypoint.h>

#define PI 3.14159265

using namespace std; 

class Navigation{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_goalpoint;
        ros::Publisher pub_is_finish;
        ros::Subscriber sub_subgoal;
        ros::Subscriber sub_pose;

        float current_pose[7];
        float current_goal[7];
        float distance_margin = 0.0;
        float heading_margin = 0.0;
        bool pose_enable = false;
        bool heading_enable = false;

    public:
        Navigation();
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void subgoalCallback(const drone_navigation::droneWaypoint::ConstPtr& msg);
        void margincheck();
        void navigation();
        void cmdRotate();
        void cmdShift();
        void cmdPose();
        float distanceP2P(float *p1, float *p2);
        float headingP2P(float *p1, float *p2);
};

Navigation :: Navigation(){
    pub_goalpoint = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    pub_is_finish = n.advertise<std_msgs::Bool>("navigation_manager/is_finish", 10);
    sub_subgoal = n.subscribe<drone_navigation::droneWaypoint>("waypoint_planner/drone_waypoint", 1,  &Navigation::subgoalCallback, this);
    sub_pose = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1,  &Navigation::positionCallback, this);
}

void Navigation :: positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose[0] = msg->pose.position.x;
    current_pose[1] = msg->pose.position.y;
    current_pose[2] = msg->pose.position.z;
    current_pose[3] = msg->pose.orientation.x;
    current_pose[4] = msg->pose.orientation.y;
    current_pose[5] = msg->pose.orientation.z;
    current_pose[6] = msg->pose.orientation.w;
    return;
}

void Navigation :: subgoalCallback(const drone_navigation::droneWaypoint::ConstPtr& msg){
    current_goal[0] = msg->pose.position.x;
    current_goal[1] = msg->pose.position.y;
    current_goal[2] = msg->pose.position.z;
    current_goal[3] = msg->pose.orientation.x;
    current_goal[4] = msg->pose.orientation.y;
    current_goal[5] = msg->pose.orientation.z;
    current_goal[6] = msg->pose.orientation.w;
    pose_enable = msg->pose_enable;
    heading_enable = msg->heading_enable;
    distance_margin = msg->distance_margin;
    heading_margin = msg->heading_margin;
    return;
}

void Navigation :: cmdRotate(){
    geometry_msgs::PoseStamped pub_msg_goal;
    pub_msg_goal.header.frame_id = "local_origin";
    pub_msg_goal.pose.position.x = current_pose[0];
    pub_msg_goal.pose.position.y = current_pose[1];
    pub_msg_goal.pose.position.z = current_pose[2];
    pub_msg_goal.pose.orientation.x = current_goal[3];
    pub_msg_goal.pose.orientation.y = current_goal[4];
    pub_msg_goal.pose.orientation.z = current_goal[5];
    pub_msg_goal.pose.orientation.w = current_goal[6];
    pub_goalpoint.publish(pub_msg_goal);
    return;
}

void Navigation :: cmdShift(){
    geometry_msgs::PoseStamped pub_msg_goal;
    pub_msg_goal.header.frame_id = "local_origin";
    pub_msg_goal.pose.position.x = current_goal[0];
    pub_msg_goal.pose.position.y = current_goal[1];
    pub_msg_goal.pose.position.z = current_goal[2];
    pub_msg_goal.pose.orientation.x = current_pose[3];
    pub_msg_goal.pose.orientation.y = current_pose[4];
    pub_msg_goal.pose.orientation.z = current_pose[5];
    pub_msg_goal.pose.orientation.w = current_pose[6];
    pub_goalpoint.publish(pub_msg_goal);
    return;
}

void Navigation :: cmdPose(){
    geometry_msgs::PoseStamped pub_msg_goal;
    pub_msg_goal.header.frame_id = "local_origin";
    pub_msg_goal.pose.position.x = current_goal[0];
    pub_msg_goal.pose.position.y = current_goal[1];
    pub_msg_goal.pose.position.z = current_goal[2];
    pub_msg_goal.pose.orientation.x = current_goal[3];
    pub_msg_goal.pose.orientation.y = current_goal[4];
    pub_msg_goal.pose.orientation.z = current_goal[5];
    pub_msg_goal.pose.orientation.w = current_goal[6];
    pub_goalpoint.publish(pub_msg_goal);
    return;
}

float Navigation :: distanceP2P(float *p1, float *p2){
    return sqrt(powf(abs(p1[0] - p2[0]), 2) + powf(abs(p1[1] - p2[1]), 2) + powf(abs(p1[2] - p2[2]), 2));
}

float Navigation :: headingP2P(float *p1, float *p2){
    double p1_rpy[3], p2_rpy[3];
    tf::Quaternion q_p1(
        p1[3],
        p1[4],
        p1[5],
        p1[6]);
    tf::Matrix3x3 m_1(q_p1);
    m_1.getRPY(p1_rpy[0], p1_rpy[1], p1_rpy[2]);

    tf::Quaternion q_p2(
        p2[3],
        p2[4],
        p2[5],
        p2[6]);
    tf::Matrix3x3 m_2(q_p2);
    m_2.getRPY(p2_rpy[0], p2_rpy[1], p2_rpy[2]);

    return abs((float)p1_rpy[2] - (float)p2_rpy[2]);
}

void Navigation :: margincheck(){
    if(~pose_enable && ~heading_enable){
        cout << "ERROR: No margin define" << endl;
    }else if(pose_enable && heading_enable){
        if(distanceP2P(current_pose, current_goal) < distance_margin && headingP2P(current_pose, current_goal) < heading_margin){
            std_msgs::Bool pub_msg_state;
            pub_msg_state.data = true;
            pub_is_finish.publish(pub_msg_state);
        }
    }else if(pose_enable){
        if(distanceP2P(current_pose, current_goal) < distance_margin){
            std_msgs::Bool pub_msg_state;
            pub_msg_state.data = true;
            pub_is_finish.publish(pub_msg_state);
        }
    }else{
        if(headingP2P(current_pose, current_goal) < heading_margin){
            std_msgs::Bool pub_msg_state;
            pub_msg_state.data = true;
            pub_is_finish.publish(pub_msg_state);
        }       
    }
    return;
}

void Navigation :: navigation(){
    margincheck();
    if(pose_enable && heading_enable){
        cmdPose();
    }else if(pose_enable){
        cmdShift();
    }else{
        cmdRotate();
    }
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "navigation");
    Navigation uav;
    while(ros::ok()){
        uav.navigation();
        ros::spinOnce();
    }
    return 0;
}