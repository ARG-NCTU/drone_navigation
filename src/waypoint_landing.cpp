#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#define PI 3.14159265

using namespace std; 

class Landing{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_goalpoint;
        ros::Publisher pub_marker;
        ros::Subscriber sub_current_pose;
        ros::Subscriber sub_land_area;

        geometry_msgs::PoseStamped pub_msg_goal;
        

        float current_pose[3] = {0.0, 0.0, 8.0};
        float land_pose[3] = {0.0, 0.0, 0.0};
        //default
        float margin_max = 10; 
        float margin_min = 0.05;
        float precise_alignment_height = 2;
        float cruise_height = 8;
        //default
    public:
        Landing();
        void getParam();
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void landingCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void landPlanner();
        float marginEval(float height);
        void landingVisualize();
        void icosagonGenerator(float *landing_spot, float margin, float **marker_array);
};

Landing :: Landing(){
    pub_goalpoint = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    pub_marker = n.advertise<visualization_msgs::MarkerArray>("margin_cage", 10);
    sub_current_pose = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1,  &Landing::positionCallback, this);
    sub_land_area = n.subscribe<geometry_msgs::PoseStamped>("/landing_area", 1,  &Landing::landingCallback, this);
}

void Landing :: positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose[0] =  msg->pose.position.x;
    current_pose[1] =  msg->pose.position.y;
    current_pose[2] =  msg->pose.position.z;
    return;
}

void Landing :: landingCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    land_pose[0] =  msg->pose.position.x;
    land_pose[1] =  msg->pose.position.y;
    land_pose[2] =  msg->pose.position.z;
    return;
}

void Landing :: landPlanner(){
    cout << "height: 08 / margin: " << marginEval(8) << endl;
    cout << "height: 07 / margin: " << marginEval(7) << endl;
    cout << "height: 06 / margin: " << marginEval(6) << endl;
    cout << "height: 05 / margin: " << marginEval(5) << endl;
    cout << "height: 04 / margin: " << marginEval(4) << endl;
    cout << "height: 03 / margin: " << marginEval(3) << endl;
    cout << "height: 02 / margin: " << marginEval(2) << endl;
    cout << "height: 01 / margin: " << marginEval(1) << endl;
    cout << "height: 00 / margin: " << marginEval(0) << endl;
    return;
}

float Landing :: marginEval(float height){
    if(height < land_pose[2]){
        cout << "ERROR: localization / landing height" << endl;
        return 0;
    }
    if(height-land_pose[2] <= precise_alignment_height){
        return margin_min;
    }
    return (margin_max-margin_min) / (cruise_height-land_pose[2]-precise_alignment_height) * (height-land_pose[2]-precise_alignment_height) + margin_min;
}

void Landing :: icosagonGenerator(float *landing_spot, float margin, float **marker_array){

}

void Landing :: landingVisualize(){
    visualization_msgs::MarkerArray pub_msg_cage;

    visualization_msgs::Marker pub_msg_line_top;
    visualization_msgs::Marker pub_msg_line_mid;
    visualization_msgs::Marker pub_msg_line_bot;
    // pub_msg_line_top.header.frame_id = pub_msg_line_mid.header.frame_id = pub_msg_line_bot.header.frame_id = "map";
    // pub_msg_line_top.header.stamp = pub_msg_line_mid.header.stamp = pub_msg_line_bot.header.stamp = ros::Time::now();
    // pub_msg_line_top.ns = pub_msg_line_mid.ns = pub_msg_line_bot.ns = "cage";
    // pub_msg_line_top.action = pub_msg_line_mid.action = pub_msg_line_bot.action = visualization_msgs::Marker::ADD;
    // pub_msg_line_top.pose.orientation.w = pub_msg_line_mid.pose.orientation.w = pub_msg_line_bot.pose.orientation.w = 1.0;
    // pub_msg_line_top.type = pub_msg_line_mid.type = pub_msg_line_bot.type = visualization_msgs::Marker::LINE_STRIP;

    // pub_msg_line_top.scale.x = pub_msg_line_mid.scale.x = pub_msg_line_bot.scale.x = 0.01;
    // pub_msg_line_top.scale.y = pub_msg_line_mid.scale.y = pub_msg_line_bot.scale.y = 0.01;
    // pub_msg_line_top.scale.z = pub_msg_line_mid.scale.z = pub_msg_line_bot.scale.z = 0.01;

    // pub_msg_line_top.color.b = pub_msg_line_mid.color.b = pub_msg_line_bot.color.b = 1.0;
    // pub_msg_line_top.color.a = pub_msg_line_mid.color.a = pub_msg_line_bot.color.a = 1.0;

    pub_msg_line_top.header.frame_id = "map";
    pub_msg_line_top.header.stamp = ros::Time::now();
    pub_msg_line_top.ns = "cage";
    pub_msg_line_top.action = visualization_msgs::Marker::ADD;
    pub_msg_line_top.pose.orientation.w = 1.0;
    pub_msg_line_top.type = visualization_msgs::Marker::LINE_STRIP;
    pub_msg_line_top.scale.x = 0.01;
    pub_msg_line_top.scale.y = 0.01;
    pub_msg_line_top.scale.z = 0.01;
    pub_msg_line_top.color.b = 1.0;
    pub_msg_line_top.color.a = 1.0;

    pub_msg_line_mid.header.frame_id = "map";
    pub_msg_line_mid.header.stamp = ros::Time::now();
    pub_msg_line_mid.ns = "cage";
    pub_msg_line_mid.action = visualization_msgs::Marker::ADD;
    pub_msg_line_mid.pose.orientation.w = 1.0;
    pub_msg_line_mid.type = visualization_msgs::Marker::LINE_STRIP;
    pub_msg_line_mid.scale.x = 0.01;
    pub_msg_line_mid.scale.y = 0.01;
    pub_msg_line_mid.scale.z = 0.01;
    pub_msg_line_mid.color.b = 1.0;
    pub_msg_line_mid.color.a = 1.0;

    pub_msg_line_bot.header.frame_id = "map";
    pub_msg_line_bot.header.stamp = ros::Time::now();
    pub_msg_line_bot.ns = "cage";
    pub_msg_line_bot.action = visualization_msgs::Marker::ADD;
    pub_msg_line_bot.pose.orientation.w = 1.0;
    pub_msg_line_bot.type = visualization_msgs::Marker::LINE_STRIP;
    pub_msg_line_bot.scale.x = 0.01;
    pub_msg_line_bot.scale.y = 0.01;
    pub_msg_line_bot.scale.z = 0.01;
    pub_msg_line_bot.color.b = 1.0;
    pub_msg_line_bot.color.a = 1.0;


    // top
    for (int i = 0; i < 21; i++){
        geometry_msgs::Point p;
        p.z = cruise_height;
        p.x = marginEval(p.z) * cos(2 * PI / 20 * i) + land_pose[0];
        p.y = marginEval(p.z) * sin(2 * PI / 20 * i) + land_pose[1];
        pub_msg_line_top.points.push_back(p);
    }
    // top

    // mid
    for (int i = 0; i < 21; i++){
        geometry_msgs::Point p;
        p.z = (cruise_height - precise_alignment_height) / 2;
        p.x = marginEval(p.z) * cos(2 * PI / 20 * i) + land_pose[0];
        p.y = marginEval(p.z) * sin(2 * PI / 20 * i) + land_pose[1];
        pub_msg_line_mid.points.push_back(p);
    }
    // mid

    // bot
    for (int i = 0; i < 21; i++){
        geometry_msgs::Point p;
        p.z = precise_alignment_height;
        p.x = marginEval(p.z) * cos(2 * PI / 20 * i) + land_pose[0];
        p.y = marginEval(p.z) * sin(2 * PI / 20 * i) + land_pose[1];
        pub_msg_line_bot.points.push_back(p);
    }
    // bot

    // pub_msg_cage.markers[0] = pub_msg_line_top;
    // pub_msg_cage.markers[1] = pub_msg_line_mid;
    // pub_msg_cage.markers[2] = pub_msg_line_bot;

    pub_msg_cage.markers.push_back(pub_msg_line_bot);
    pub_msg_cage.markers.push_back(pub_msg_line_mid);
    pub_msg_cage.markers.push_back(pub_msg_line_top);

    // pub_msg_cage.markers.insert(pub_msg_cage.markers.end(), pub_msg_line_top.begin(), pub_msg_line_top.end());
    // pub_msg_cage.markers.insert(pub_msg_cage.markers.end(), pub_msg_line_mid.begin(), pub_msg_line_mid.end());
    // pub_msg_cage.markers.insert(pub_msg_cage.markers.end(), pub_msg_line_bot.begin(), pub_msg_line_bot.end());
    pub_marker.publish(pub_msg_cage);
    return;
}



void Landing :: getParam(){
    n.getParam("/landing/margin_max", margin_max);
    n.getParam("/landing/margin_min", margin_min);
    n.getParam("/landing/precise_alignment_height", precise_alignment_height);
    n.getParam("/landing/cruise_height", cruise_height);
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "waypoint_landing");
    Landing uav;
    uav.getParam();
    ros::Rate r(30);
    uav.landPlanner();
    uav.landingVisualize();
    while(ros::ok()){
        
        uav.landingVisualize();
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
