#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


class Waypoint{
    private:
    ros::NodeHandle n;
    ros::Publisher msg_pub;
    ros::Subscriber msg_sub;
    int desired_waypoint[3][2] = {
        {10, 20},
        {10, 30},
        {10, 10},
    };
    int state_num = 0;
    int state_now = 0;
    int seq = 1;
    bool init = false;
    float margin = 1.0;
    public: 
    Waypoint(){
        msg_sub = n.subscribe("mavros/local_position/pose", 1000,  &Waypoint::positionCallback, this);
        msg_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
        state_num = sizeof(desired_waypoint) / sizeof(*desired_waypoint);
        ROS_INFO("number of waypoint: %d\n", state_num);
    }

    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& input){
        if(state_now == state_num){
            ros::shutdown();
            return;
        }
        geometry_msgs::PoseStamped position_new;
        position_new.header.seq = seq;
        position_new.header.stamp = ros::Time::now();
        position_new.header.frame_id = "local_origin";
        
        if(state_now == 0 && init == false){
            init = true;
            position_new.pose.position.x = desired_waypoint[state_now][0];
            position_new.pose.position.y = desired_waypoint[state_now][1];
            msg_pub.publish(position_new);
            seq++;
        }
        
        if (sqrt(pow(abs(desired_waypoint[state_now][0] - input->pose.position.x), 2) + 
                pow(abs(desired_waypoint[state_now][1] - input->pose.position.y), 2)) < margin){
            state_now++;
            ROS_INFO("arrived waypoint: %d / %d\n", state_now, state_num);
            position_new.pose.position.x = desired_waypoint[state_now][0];
            position_new.pose.position.y = desired_waypoint[state_now][1];
            msg_pub.publish(position_new);
            seq++;
        }
        return;
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "drone_nav");
    Waypoint waypoint;
    ros::spin();
    return 0;
}