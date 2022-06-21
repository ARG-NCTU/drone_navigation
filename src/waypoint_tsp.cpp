#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros_tsp/tspGoalpoint.h"
using namespace std;
ros_tsp::tspGoalpoint msg;

class Waypoint{
    private:
    ros::NodeHandle n;
    ros::Publisher msg_pub_subgoal;
    ros::Subscriber msg_sub_tsp;
    ros::Subscriber msg_sub_pose;
    // int desired_waypoint[3][2] = {
    //     {10, 20},
    //     {10, 30},
    //     {10, 10},
    // };
    int state_num = 0;
    int state_now = 0;
    int seq = 1;
    bool init = false;
    float margin = 1.0;
    float** desired_waypoint;

    public: 
    Waypoint(){
        msg_sub_tsp = n.subscribe("tsp_goalpoint", 1000,  &Waypoint::waypointCallback, this);
        msg_sub_pose = n.subscribe("mavros/local_position/pose", 1000,  &Waypoint::positionCallback, this);
        msg_pub_subgoal = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
        // state_num = sizeof(desired_waypoint) / sizeof(*desired_waypoint);
    }
    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& input){
        geometry_msgs::PoseStamped position_new;
        position_new.header.frame_id = "local_origin";
        if(init == false){return;}
        if(state_now == state_num){
            while(ros::ok()){
                ROS_INFO("TSP Finished");
                position_new.header.seq = seq;
                position_new.header.stamp = ros::Time::now();
                position_new.pose.position.x = desired_waypoint[state_now][0];
                position_new.pose.position.y = desired_waypoint[state_now][1];
                msg_pub_subgoal.publish(position_new);
                seq++;
            }
            return;
        }
        
        if (sqrt(pow(abs(desired_waypoint[state_now][0] - input->pose.position.x), 2) + 
                pow(abs(desired_waypoint[state_now][1] - input->pose.position.y), 2)) < margin){
            state_now++;
            ROS_INFO("arrived waypoint: %d / %d\n", state_now, state_num);
            position_new.pose.position.x = desired_waypoint[state_now][0];
            position_new.pose.position.y = desired_waypoint[state_now][1];
            msg_pub_subgoal.publish(position_new);
            seq++;
        }
        return;
    }

    void waypointCallback(const ros_tsp::tspGoalpoint::ConstPtr& input){
        if(init == false){
            init = true;
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
        // for(int i=0; i<state_num; i++){
        //     for(int j=0; j<2; j++){
        //         cout << desired_waypoint[i][j] << " ";
        //     }
        //     cout << endl;
        // }
        // cout <<"size : "<< sizeof(desired_waypoint) / sizeof(*desired_waypoint) << endl;
        // cout <<"size : "<< sizeof(desired_waypoint[0]) / sizeof(float) << endl;        
        return;
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "drone_nav");
    Waypoint waypoint;
    ros::spin();
    return 0;
}