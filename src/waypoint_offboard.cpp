#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
using namespace std; 

class Offboard{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_goalpoint;
        ros::Subscriber sub_position;
        ros::Subscriber sub_state;
        ros::ServiceClient client_mode;
        ros::ServiceClient client_arming;

        mavros_msgs::State current_state;
        geometry_msgs::PoseStamped pub_msg_goal;
        float current_position[3] = {0.0, 0.0, 0.0};
        float margin = 0.08;
    public:
        Offboard();
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void stateCallback(const mavros_msgs::State::ConstPtr& msg);
        void clientMode(string input);
        void clientArm();
        int navigation(int stage, float waypoint[][3]);
        void navScript(int mode);
        float distanceP2P(float *p1, float *p2);
};

Offboard :: Offboard(){
    pub_goalpoint = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    sub_position = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1,  &Offboard::positionCallback, this);
    sub_state = n.subscribe<mavros_msgs::State>("mavros/state", 1,  &Offboard::stateCallback, this);
    client_mode = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    client_arming = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    
}

void Offboard :: positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position[0] =  msg->pose.position.x;
    current_position[1] =  msg->pose.position.y;
    current_position[2] =  msg->pose.position.z;
    return;
}

void Offboard :: stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    return;
}

void Offboard :: clientMode(string input){
    mavros_msgs::SetMode srv_msg;
    srv_msg.request.custom_mode = input;
    client_mode.call(srv_msg);
    return;
}

void Offboard :: clientArm(){
    mavros_msgs::CommandBool srv_msg;
    srv_msg.request.value = true;
    client_arming.call(srv_msg);
    return;
}

float Offboard :: distanceP2P(float *p1, float *p2){

    // cout << *(&p1+1)-p1 << " " << *(&p2+1)-p2 << endl;
    // if((*(&p1+1)-p1 != 3) || (*(&p2+1)-p2 != 3)){
    //     return -1;
    // }
    return sqrt(powf(abs(p1[0] - p2[0]), 2) + powf(abs(p1[1] - p2[1]), 2) + powf(abs(p1[2] - p2[2]), 2));
}

int Offboard :: navigation(int stage, float waypoint[][3]){
    float *goal = waypoint[stage];
    cout << "  dist: " << distanceP2P(goal, current_position) << endl;
    if(distanceP2P(goal, current_position) < margin){
        return stage+1;
    }else{
        pub_msg_goal.header.frame_id = "local_origin";
        pub_msg_goal.pose.position.x = waypoint[stage][0];
        pub_msg_goal.pose.position.y = waypoint[stage][1];
        pub_msg_goal.pose.position.z = waypoint[stage][2];
        pub_goalpoint.publish(pub_msg_goal);
    }
    return stage;
}



int main(int argc, char **argv){
    ros::init(argc, argv, "waypoint_offboard");
    Offboard nav;
    int state = 0;
    float square_length = 3;
    float waypoint[12][3] = {
        {0, 0, 0},
        {0, 0, 3},
        {square_length/2, 0, 3},
        {square_length/2, square_length/2, 3},
        {0, square_length/2, 3},
        {-square_length/2, square_length/2, 3},
        {-square_length/2, 0, 3},
        {-square_length/2, -square_length/2, 3},
        {0, -square_length/2, 3},
        {square_length/2, -square_length/2, 3},
        {square_length/2, 0, 3},
        {0, 0, 3}
    };
    
    while(ros::ok()){
        cout << "state: " << state << "/" << sizeof(waypoint)/sizeof(*waypoint);
        state = nav.navigation(state, waypoint);
        ros::spinOnce();
    }
    return 0;
}


	

