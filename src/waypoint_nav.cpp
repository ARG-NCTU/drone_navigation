#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class Waypoint{
    private:
        ros::NodeHandle n;
        ros::Publisher msg_pub;
        ros::Subscriber msg_sub;
        ros::Publisher vis_pub;
        int desired_waypoint[4][3] = {
            {-2, 5, 10},
            {-9, 8, 10},
            {-12, 3, 10},
            {-16, 1, 10}
        };
        float current_pose[3] = {0, 0, 0};
        int state_num = 0;
        int state_now = 0;
        int seq = 1;
        bool init = false;
        float margin = 1.0;
    public:
        bt::Action action_navigation;
        Waypoint() : action_navigation("Destinate Landing"){
            msg_sub = n.subscribe("mavros/local_position/pose", 1000,  &Waypoint::positionCallback, this);
            msg_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);
            state_num = sizeof(desired_waypoint) / sizeof(*desired_waypoint);
            
        }

        void actionSet(int state){
            switch(state){
                case 1:
                    action_navigation.set_success();
                    break;
                case 0:
                    action_navigation.set_running();
                    break;
                case -1:
                    action_navigation.set_failure();
                    break;
            }
            action_navigation.publish();
            return;
        }

        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& input){
            current_pose[0] = input->pose.position.x;
            current_pose[1] = input->pose.position.y;
            current_pose[2] = input->pose.position.z;
            return;
        }
        void navigation(){
            // if(state_now == state_num){
            //     // ros::shutdown();
            //     actionSet(1);
            //     return;
            // }
            //
            if (sqrt(pow(abs(desired_waypoint[state_num-1][0] - current_pose[0]), 2) + 
                    pow(abs(desired_waypoint[state_num-1][1] - current_pose[1]), 2)) < margin){
                // ros::shutdown();
                //actionSet(1);
                return;
            }
            //actionSet(0);
            geometry_msgs::PoseStamped position_new;
            position_new.header.seq = seq;
            position_new.header.stamp = ros::Time::now();
            position_new.header.frame_id = "local_origin";
            position_new.pose.position.x = desired_waypoint[state_now][0];
            position_new.pose.position.y = desired_waypoint[state_now][1];
            position_new.pose.position.z = 10;
            msg_pub.publish(position_new);
            seq++;
            //

            // if(state_now == 0 && init == false){
            //     init = true;
            //     position_new.pose.position.x = desired_waypoint[state_now][0];
            //     position_new.pose.position.y = desired_waypoint[state_now][1];
            //     position_new.pose.position.z = 10;
            //     msg_pub.publish(position_new);
            //     seq++;
            // }
            if (sqrt(pow(abs(desired_waypoint[state_now][0] - current_pose[0]), 2) + 
                    pow(abs(desired_waypoint[state_now][1] - current_pose[1]), 2)) < margin){
                state_now++;
                ROS_INFO("arrived waypoint: %d / %d\n", state_now, state_num);
                // position_new.pose.position.x = desired_waypoint[state_now][0];
                // position_new.pose.position.y = desired_waypoint[state_now][1];
                // position_new.pose.position.z = 10;
                // msg_pub.publish(position_new);
                // seq++;
            }
            return;
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "drone_nav");
    Waypoint waypoint;
    while(ros::ok()){
        if(waypoint.action_navigation.is_active() && waypoint.action_navigation.active_has_changed()){
            ROS_INFO("Action: Nav activiate");
        }
        if(waypoint.action_navigation.active_has_changed() && !(waypoint.action_navigation.is_active())){
            ROS_INFO("done");
            waypoint.actionSet(1);
        }
        if(waypoint.action_navigation.is_active()){
            waypoint.actionSet(0);
            waypoint.navigation();
        }
        ros::spinOnce();
    }
    return 0;
}