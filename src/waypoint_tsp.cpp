#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include "ros_tsp/tspGoalpoint.h"
#include <behavior_tree/behavior_tree.h>
using namespace std;
ros_tsp::tspGoalpoint msg;

class Waypoint{
    private:
        ros::NodeHandle n;
        ros::Publisher msg_pub_subgoal;
        ros::Publisher msg_pub_state;
        ros::Subscriber msg_sub_tsp;
        ros::Subscriber msg_sub_pose;

        int state_num = 0;
        int state_now = 0;
        int seq = 1;
        bool init_tsp = false;
        bool state_tsp = false;
        float margin = 1.0;
        float** desired_waypoint;
        float desired_height = 0;
        float current_pose[3] = {0, 0, 0};
    public: 
        bt::Action action_tsp;
        Waypoint();
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& input);
        void waypointCallback(const ros_tsp::tspGoalpoint::ConstPtr& input);
        void actionSet(int state);
        void navigation();
        void stateCheck();
};

Waypoint :: Waypoint() : action_tsp("Destinate TSP"){
    msg_sub_tsp = n.subscribe("tsp_goalpoint", 1000,  &Waypoint::waypointCallback, this);
    msg_sub_pose = n.subscribe("mavros/local_position/pose", 1000,  &Waypoint::positionCallback, this);
    msg_pub_subgoal = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
    msg_pub_state = n.advertise<std_msgs::Bool>("tsp_finished", 10);
}

void Waypoint :: positionCallback(const geometry_msgs::PoseStamped::ConstPtr& input){
    current_pose[0] = input->pose.position.x;
    current_pose[1] = input->pose.position.y;
    current_pose[2] = input->pose.position.z;
    n.getParam("drone/desired_height", desired_height);
    return;
}

void Waypoint :: waypointCallback(const ros_tsp::tspGoalpoint::ConstPtr& input){
    if(init_tsp == false){
        init_tsp = true;
        state_num = input->data_x.size();
        ROS_INFO("number of waypoint: %d\n", state_num);
        desired_waypoint = new float*[state_num];
        for(int i=0; i<state_num; i++){
            desired_waypoint[i] = new float[2];
        }
        for(int i=0; i<state_num; i++){
            desired_waypoint[i][0] = input->data_x[i];
            desired_waypoint[i][1] = input->data_y[i];
        }
    }
    return;
}



void Waypoint :: actionSet(int state){
    switch(state){
        case 1:
            action_tsp.set_success();
            break;
        case 0:
            action_tsp.set_running();
            break;
        case -1:
            action_tsp.set_failure();
            break;
    }
    action_tsp.publish();
    return;
}

void Waypoint :: navigation(){
    if(init_tsp == false){return;}
    if(state_now == state_num){
        state_tsp = true;
        actionSet(1);
        return;
    }
    geometry_msgs::PoseStamped position_new;
    position_new.header.seq = seq;
    position_new.header.stamp = ros::Time::now();
    position_new.header.frame_id = "local_origin";
    position_new.pose.position.x = desired_waypoint[state_now][0];
    position_new.pose.position.y = desired_waypoint[state_now][1];
    position_new.pose.position.z = desired_height;
    msg_pub_subgoal.publish(position_new);
    seq++;
    
    if (sqrt(pow(abs(desired_waypoint[state_now][0] - current_pose[0]), 2) + 
            pow(abs(desired_waypoint[state_now][1] - current_pose[1]), 2)) < margin){
        state_now++;
        ROS_INFO("arrived waypoint: %d / %d\n", state_now, state_num);
    }
    return;
}

void Waypoint :: stateCheck(){
    std_msgs::Bool tsp_state;
    tsp_state.data = state_tsp;
    msg_pub_state.publish(tsp_state);
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "drone_tsp");
    Waypoint waypoint;
    while(ros::ok()){
        if(waypoint.action_tsp.is_active() && waypoint.action_tsp.active_has_changed()){
            ROS_INFO("Action: Nav TSP activiate");
        }
        if(waypoint.action_tsp.active_has_changed() && !(waypoint.action_tsp.is_active())){
            ROS_INFO("Action: Done");
            waypoint.actionSet(1);
        }
        if(waypoint.action_tsp.is_active()){
            waypoint.actionSet(0);
            waypoint.navigation();
            //ros::Duration(0.5).sleep(); 
        }
        waypoint.stateCheck();
        ros::spinOnce();
    }
    return 0;
}