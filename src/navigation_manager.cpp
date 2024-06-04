#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <behavior_tree/behavior_tree.h>
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
        ros::Subscriber sub_height_offset;
        ros::Subscriber sub_twist;

        float drone_origin_pose[7] = {0, 0, 0, 0, 0, 0, 0};
        float current_pose[7] = {0, 0, 0, 0, 0, 0, 0};
        float current_twist[6] = {0, 0, 0, 0, 0, 0};
        float last_goal[7] = {-1, -1, -1, -1, -1, -1, -1};
        float current_goal[7] = {0, 0, 0, 0, 0, 0, 0};
        float distance_margin = 0.8;
        float heading_margin = 0.017;
        float height_offset = 0;
        unsigned int planner_seq = 0;
        string planner_name = "none";
        bool height_estimate_enable = false;
        bool pose_enable = false;
        bool rotate_enable = false;
        bool heading_enable = false;
        bool last_waypoint = false;
        bool drone_as_origin = false;
        bool condition_status = true;

        bool one_time = false;

        // for keep last pose
        bool store_keep_pose = false;
        float keep_pose[7] = {0, 0, 0, 0, 0, 0, 0};

        //check flag for all subgoal are finished keep last goal
        bool all_subgoal_finished = false;


    public:
        bt::Condition condition;
        Navigation();
        void getParam();
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void heightCallback(const std_msgs::Float32::ConstPtr& msg);
        void subgoalCallback(const drone_navigation::droneWaypoint::ConstPtr& msg);
        void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
        
        void euler2quaternion(float yaw, float *x, float *y, float *z, float *w);
        void quaternion2euler(float *roll, float *pitch, float *yaw, float x, float y, float z, float w);
        float distanceP2P(float *p1, float *p2);
        float headingP2P(float *p1, float *p2);
        
        bool twistCheck();
        void marginCheck();
        void navigation();
        void cmdRotate();
        void cmdShift();
        void cmdPose();
        void cmdKeep();

        void conditionSet(bool state);
};

Navigation :: Navigation() : condition("navigation_running"){
    pub_goalpoint = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    pub_is_finish = n.advertise<std_msgs::Bool>("navigation_manager/is_finish", 10);
    sub_subgoal = n.subscribe<drone_navigation::droneWaypoint>("waypoint_planner/drone_waypoint", 1,  &Navigation::subgoalCallback, this);
    sub_pose = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1,  &Navigation::positionCallback, this);
    sub_height_offset = n.subscribe<std_msgs::Float32>("height_offset", 1,  &Navigation::heightCallback, this);
    //sub_height_offset = n.subscribe<std_msgs::Float32>("height_offset", 1,  &Navigation::heightCallback, this);
    sub_twist = n.subscribe<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 1,  &Navigation::twistCallback, this);
}


void Navigation :: getParam(){
    string node_ns = ros::this_node::getName();
    n.getParam("/" + node_ns + "/height_estimate_enable", height_estimate_enable);
    return;
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

void Navigation :: heightCallback(const std_msgs::Float32::ConstPtr& msg){
    if(height_estimate_enable){
        height_offset = msg->data;
    }
    return;
}

void Navigation :: twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
    current_twist[0] = msg->linear.x;
    current_twist[1] = msg->linear.y;
    current_twist[2] = msg->linear.z;
    current_twist[3] = msg->angular.x;
    current_twist[4] = msg->angular.y;
    current_twist[5] = msg->angular.z;

    // cout << "debug:" << endl;
    // for(int i = 0; i < 6; i++){ 
    //     cout << current_twist[i] << endl;
    // }

    return;
}

void Navigation :: subgoalCallback(const drone_navigation::droneWaypoint::ConstPtr& msg){
    // NEW
    one_time = true;

    pose_enable = msg->pose_enable;
    rotate_enable = msg->rotate_enable;
    heading_enable = msg->heading_enable;
    last_waypoint = msg->last_waypoint;
    distance_margin = msg->distance_margin;
    heading_margin = msg->heading_margin;
    if((msg->origin == (string)"drone" && rotate_enable) || (heading_enable && rotate_enable)){
        // if(!drone_as_origin){
        //     drone_as_origin = true;
        //     for(int i = 0; i < 7; i++){ drone_origin_pose[i] = current_pose[i]; }
        // }
        if((msg->planner_name != planner_name) || (msg->planner_seq != planner_seq)){
            planner_name = msg->planner_name;
            planner_seq = msg->planner_seq;
            for(int i = 0; i < 7; i++){ drone_origin_pose[i] = current_pose[i]; }
        }
        if(heading_enable){
            float origin_r, origin_p, origin_y, rotate_y, new_x, new_y, new_z, new_w;
            quaternion2euler(&origin_r, &origin_p, &origin_y, 
                            drone_origin_pose[3], 
                            drone_origin_pose[4], 
                            drone_origin_pose[5], 
                            drone_origin_pose[6]);
            float v1[2], v2[2];
            v1[0] = 1; 
            v1[1] = 0;
            v2[0] = current_goal[0] - current_pose[0];
            v2[1] = current_goal[1] - current_pose[1];
            rotate_y = acos(v2[0] / sqrt(powf(v2[0], 2) + powf(v2[1], 2)));
            // cout << "heading angle: " << (rotate_y) * 180 / PI << endl;
            euler2quaternion(rotate_y, 
                            &new_x, 
                            &new_y, 
                            &new_z, 
                            &new_w);
            current_goal[0] = drone_origin_pose[0] + msg->pose.position.x;
            current_goal[1] = drone_origin_pose[1] + msg->pose.position.y;
            current_goal[2] = drone_origin_pose[2] + msg->pose.position.z;
            current_goal[3] = new_x;
            current_goal[4] = new_y;
            current_goal[5] = new_z;
            current_goal[6] = new_w;
        }else{
            float origin_r, origin_p, origin_y, cmd_r, cmd_p, cmd_y, new_x, new_y, new_z, new_w;
            quaternion2euler(&origin_r, &origin_p, &origin_y, 
                            drone_origin_pose[3], 
                            drone_origin_pose[4], 
                            drone_origin_pose[5], 
                            drone_origin_pose[6]);
            quaternion2euler(&cmd_r, &cmd_p, &cmd_y, 
                            msg->pose.orientation.x, 
                            msg->pose.orientation.y, 
                            msg->pose.orientation.z, 
                            msg->pose.orientation.w);
            euler2quaternion(origin_y + cmd_y, 
                            &new_x, 
                            &new_y, 
                            &new_z, 
                            &new_w);
            if(msg->pose.position.x == 0){
                if(msg->pose.position.y >= 0){
                    current_goal[0] = drone_origin_pose[0] + 
                                sqrt(powf(msg->pose.position.y, 2)) * cos(origin_y + PI / 2);
                    current_goal[1] = drone_origin_pose[1] + 
                                sqrt(powf(msg->pose.position.y, 2)) * sin(origin_y + PI / 2);
                }else{
                    current_goal[0] = drone_origin_pose[0] + 
                                sqrt(powf(msg->pose.position.y, 2)) * cos(origin_y - PI / 2);
                    current_goal[1] = drone_origin_pose[1] + 
                                sqrt(powf(msg->pose.position.y, 2)) * sin(origin_y - PI / 2);
                }
            }else{
                current_goal[0] = drone_origin_pose[0] + 
                                sqrt(powf(msg->pose.position.x, 2) + powf(msg->pose.position.y, 2)) * 
                                cos(origin_y + atan(msg->pose.position.y / msg->pose.position.x));
                current_goal[1] = drone_origin_pose[1] + 
                                sqrt(powf(msg->pose.position.x, 2) + powf(msg->pose.position.y, 2)) * 
                                sin(origin_y + atan(msg->pose.position.y / msg->pose.position.x));
            }
            current_goal[2] = drone_origin_pose[2] + msg->pose.position.z;
            current_goal[3] = new_x;
            current_goal[4] = new_y;
            current_goal[5] = new_z;
            current_goal[6] = new_w;
        }
    }else if(msg->origin == (string)"local_origin"){
        if(drone_as_origin){
            drone_as_origin = false;
        }
        current_goal[0] = msg->pose.position.x;
        current_goal[1] = msg->pose.position.y;
        current_goal[2] = msg->pose.position.z;
        current_goal[3] = msg->pose.orientation.x;
        current_goal[4] = msg->pose.orientation.y;
        current_goal[5] = msg->pose.orientation.z;
        current_goal[6] = msg->pose.orientation.w;
        planner_name = msg->planner_name;
        planner_seq = msg->planner_seq;
    }
    
    // if(last_waypoint){
    //     drone_as_origin = false;
    // }
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
    double roll, pitch, yaw;
    tf::Quaternion q(
        current_pose[3],
        current_pose[4],
        current_pose[5],
        current_pose[6]);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    tf::Quaternion q_new;
    q_new.setRPY(0, 0, yaw);
    q_new = q_new.normalize();
    
    geometry_msgs::PoseStamped pub_msg_goal;
    pub_msg_goal.header.frame_id = "local_origin";
    pub_msg_goal.pose.position.x = current_goal[0];
    pub_msg_goal.pose.position.y = current_goal[1];
    pub_msg_goal.pose.position.z = current_goal[2] + height_offset;
    pub_msg_goal.pose.orientation.x = q_new.getX();
    pub_msg_goal.pose.orientation.y = q_new.getY();
    pub_msg_goal.pose.orientation.z = q_new.getZ();
    pub_msg_goal.pose.orientation.w = q_new.getW();
    pub_goalpoint.publish(pub_msg_goal);
    return;
}

void Navigation :: cmdPose(){
    geometry_msgs::PoseStamped pub_msg_goal;
    pub_msg_goal.header.frame_id = "local_origin";
    pub_msg_goal.pose.position.x = current_goal[0];
    pub_msg_goal.pose.position.y = current_goal[1];
    pub_msg_goal.pose.position.z = current_goal[2] + height_offset;
    pub_msg_goal.pose.orientation.x = current_goal[3];
    pub_msg_goal.pose.orientation.y = current_goal[4];
    pub_msg_goal.pose.orientation.z = current_goal[5];
    pub_msg_goal.pose.orientation.w = current_goal[6];
    pub_goalpoint.publish(pub_msg_goal);
    return;
}

void Navigation :: cmdKeep(){
    double roll, pitch, yaw;
    tf::Quaternion q(
        keep_pose[3],
        keep_pose[4],
        keep_pose[5],
        keep_pose[6]);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    tf::Quaternion q_new;
    q_new.setRPY(0, 0, yaw);
    q_new = q_new.normalize();
    
    geometry_msgs::PoseStamped pub_msg_goal;
    pub_msg_goal.header.frame_id = "local_origin";
    pub_msg_goal.pose.position.x = keep_pose[0];
    pub_msg_goal.pose.position.y = keep_pose[1];
    pub_msg_goal.pose.position.z = keep_pose[2];
    pub_msg_goal.pose.orientation.x = q_new.getX();
    pub_msg_goal.pose.orientation.y = q_new.getY();
    pub_msg_goal.pose.orientation.z = q_new.getZ();
    pub_msg_goal.pose.orientation.w = q_new.getW();
    pub_goalpoint.publish(pub_msg_goal);
    return;
}


void Navigation :: euler2quaternion(float yaw, float *x, float *y, float *z, float *w){
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    q = q.normalize();
    *x = q.getX();
    *y = q.getY();
    *z = q.getZ();
    *w = q.getW();
    return;
}

void Navigation :: quaternion2euler(float *roll, float *pitch, float *yaw, float x, float y, float z, float w){
    double tmp_r, tmp_p, tmp_y;
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(tmp_r, tmp_p, tmp_y);
    *roll = (float)tmp_r;
    *pitch = (float)tmp_p;
    *yaw = (float)tmp_y;
    return;
}

float Navigation :: distanceP2P(float *p1, float *p2){
    return sqrt(powf(abs(p1[0] - p2[0]), 2) + powf(abs(p1[1] - p2[1]), 2) + powf(abs(p1[2] - (p2[2] + height_offset)), 2));
}

float Navigation :: headingP2P(float *p1, float *p2){
    float p1_r, p1_p, p1_y, p2_r, p2_p, p2_y;
    quaternion2euler(&p1_r, &p1_p, &p1_y, p1[3], p1[4], p1[5], p1[6]);
    quaternion2euler(&p2_r, &p2_p, &p2_y, p2[3], p2[4], p2[5], p2[6]);
    return abs(p1_y - p2_y);
}

bool Navigation :: twistCheck(){
    // cout << "debug:" << endl;
    // for(int i = 0; i < 6; i++){ 
    //     cout << current_twist[i] << endl;
    // }
    int checkbit = 0;
    for(int i = 0; i < 6; i++){ if(current_twist[i] != 0){ checkbit++; } }
    if(checkbit > 0){
        store_keep_pose = false;
        return true;
    }else{
        // If no manaul control detected, maintain the current pose
        if (!store_keep_pose){
            for(int i = 0; i < 7; i++){keep_pose[i] = current_pose[i];}
            store_keep_pose = true;

        }
        return false;
    }
}

void Navigation :: marginCheck(){ // need to consider height offset
    int checkbit = 0;

    for(int i = 0; i < 7; i++){ if(last_goal[i] != current_goal[i]){ checkbit++; } }
    if(last_waypoint && all_subgoal_finished && checkbit == 0){//new condition to keep last goal and can switch to mannual control last point
        ROS_INFO("ALL SUBGOAL FINISHED");
        pose_enable = false;
        rotate_enable = false;
        return;}

    all_subgoal_finished = false;

    if (checkbit==0){ return;}

    cout << "DIST current: " << distanceP2P(current_pose, current_goal) << endl;
    cout << "DIST margin : " << distance_margin << endl;
    cout << "HEAD current: " << headingP2P(current_pose, current_goal) << endl;
    cout << "HEAD margin : " << heading_margin << endl;

    bool withinDistanceMargin = distanceP2P(current_pose, current_goal) < distance_margin;
    bool withinHeadingMargin = headingP2P(current_pose, current_goal) < heading_margin;

    if ((!pose_enable && !rotate_enable) || (!withinDistanceMargin && !withinHeadingMargin)) {
        return;  // No need to proceed if no movement is enabled or margins aren't met
    }

    for (int i = 0; i < 7; i++) {
        last_goal[i] = current_goal[i];
    }

    // Publish that the current goal is achieved
    std_msgs::Bool pub_msg_state;
    pub_msg_state.data = true;
    pub_is_finish.publish(pub_msg_state);
    ROS_INFO("tick:   %s", planner_name.c_str());

    // Check if this is the last waypoint and update condition status
    if (last_waypoint) {
        ROS_INFO("finish: %s", planner_name.c_str());
        condition_status = false;

        //store the last goal to keep pose
        for(int i = 0; i < 7; i++){keep_pose[i] = current_goal[i];}
        all_subgoal_finished = true;
        
    }



    // if(!pose_enable && !rotate_enable){
    //     return;
    // }else if(pose_enable && rotate_enable){
    //     if(checkbit > 0){ condition_status = true; }
    //         cout << "DIST current: " << distanceP2P(current_pose, current_goal) << endl;
    //         cout << "DIST margin : " << distance_margin << endl;
    //         cout << "HEAD current: " << headingP2P(current_pose, current_goal) << endl;
    //         cout << "HEAD margin : " << heading_margin << endl;
    //     if(distanceP2P(current_pose, current_goal) < distance_margin 
    //         && headingP2P(current_pose, current_goal) < heading_margin
    //         && checkbit > 0){
    //         for(int i = 0; i < 7; i++){ last_goal[i] = current_goal[i]; }
    //         std_msgs::Bool pub_msg_state;
    //         pub_msg_state.data = true;
    //         pub_is_finish.publish(pub_msg_state);
    //         ROS_INFO("tick:   %s", planner_name.c_str());
    //         if(last_waypoint)
    //         { ROS_INFO("finish: %s", planner_name.c_str()); condition_status = false;}
    //     }
    // }else if(pose_enable){
    //     if(checkbit > 0){ condition_status = true; }
    //     if(distanceP2P(current_pose, current_goal) < distance_margin
    //         && checkbit > 0){
    //         for(int i = 0; i < 7; i++){ last_goal[i] = current_goal[i]; }
    //         std_msgs::Bool pub_msg_state;
    //         pub_msg_state.data = true;
    //         pub_is_finish.publish(pub_msg_state);
    //         ROS_INFO("tick:   %s", planner_name.c_str());
    //         if(last_waypoint)
    //         { ROS_INFO("finish: %s", planner_name.c_str()); condition_status = false;}
    //     }
    // }else if(rotate_enable){
    //     if(checkbit > 0){ condition_status = true; }
    //     if(headingP2P(current_pose, current_goal) < heading_margin
    //         && checkbit > 0){
    //         for(int i = 0; i < 7; i++){ last_goal[i] = current_goal[i]; }
    //         std_msgs::Bool pub_msg_state;
    //         pub_msg_state.data = true;
    //         pub_is_finish.publish(pub_msg_state);
    //         ROS_INFO("tick:   %s", planner_name.c_str());
    //         if(last_waypoint)
    //         { ROS_INFO("finish: %s", planner_name.c_str()); condition_status = false;}
    //     }
    // }
    return;
}

void Navigation :: conditionSet(bool state){
    condition.set(state);
    condition.publish();
    return;
}

void Navigation::navigation() {
    ros::Rate rate(30);
    marginCheck();

    if (!twistCheck()) { // No human control
        ROS_INFO("Controller is not activated...Enable Position Navigation");
        ROS_INFO("pose_enable: %d", pose_enable);
        ROS_INFO("rotate_enable: %d", rotate_enable);
        ROS_INFO("last_waypoint: %d", last_waypoint);
        ROS_INFO("Current Pose: [%f, %f, %f, %f, %f, %f, %f]", current_pose[0], current_pose[1], current_pose[2], current_pose[3], current_pose[4], current_pose[5], current_pose[6]);
        ROS_INFO("Current Goal: [%f, %f, %f, %f, %f, %f, %f]", current_goal[0], current_goal[1], current_goal[2], current_goal[3], current_goal[4], current_goal[5], current_goal[6]);
        ROS_INFO("keep_pose: [%f, %f, %f, %f, %f, %f, %f]", keep_pose[0], keep_pose[1], keep_pose[2], keep_pose[3], keep_pose[4], keep_pose[5], keep_pose[6]);
        if (pose_enable || rotate_enable) {
            // 2-1: Waypoint from subgoalCallback and margin not satisfied
            ROS_INFO("Moving to subgoal waypoint");
            if (pose_enable && rotate_enable) {
                cmdPose();
            } else if (pose_enable) {
                cmdShift();
            } else if (rotate_enable) {
                cmdRotate();
            }
        } else {
            // 2-2: Maintain or resume last known stable position using keep_pose (maybe the last goal point or the human control last point)  
            ROS_INFO("No active waypoint commands; maintaining position using keep_pose");
            cmdKeep();
        }
    } else {
        // 1: Human control detected
        ROS_INFO("Controller is activated...Disable Position Navigation");
    }
    conditionSet(condition_status);
    rate.sleep();
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "navigation");
    
    Navigation uav;
    uav.getParam();
    while(ros::ok()){
        uav.navigation();
        ros::spinOnce();
    }
    return 0;
}