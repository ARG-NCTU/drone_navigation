#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
using namespace std; 


class Visualize{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_pose;
        ros::Publisher pub_marker_pose;

        geometry_msgs::PoseStamped current_pose;

        int m_index = 0;

    public:
        Visualize();
        void dronePositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void pathVisualize();
};

Visualize :: Visualize(){
    sub_pose = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1,  &Visualize::dronePositionCallback, this);
    pub_marker_pose = n.advertise<visualization_msgs::MarkerArray>("drone_marker/path", 10);
}

void Visualize :: dronePositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
    return;
}

void Visualize :: pathVisualize(){
    ros::Rate rate(5);

    visualization_msgs::MarkerArray pub_visual_path;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "local_origin";
    marker.header.stamp = ros::Time::now();
    marker.ns = "path";
    marker.id = m_index;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    // yellow  
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1.0;
    marker.pose.position.x = current_pose.pose.position.x;
    marker.pose.position.y = current_pose.pose.position.y;
    marker.pose.position.z = 0;
    pub_visual_path.markers.push_back(marker);
    
    m_index ++;
    pub_marker_pose.publish(pub_visual_path);
    rate.sleep();
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "visualize_path");
    Visualize path;
    while(ros::ok()){
        path.pathVisualize();
        ros::spinOnce();
    }
    return 0;
}