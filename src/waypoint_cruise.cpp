#include <iostream>
#include <string> 
#include <list>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <behavior_tree/behavior_tree.h>
#include <drone_navigation/droneWaypoint.h>

using namespace std; 


class Waypoint{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_subgoal;
        ros::Subscriber sub_state;
        ros::Subscriber sub_goal;

        geometry_msgs::PoseStamped pub_msg_goal;

        XmlRpc::XmlRpcValue xml_waypoint;

        float **waypoint_list;

        
        list<drone_navigation::droneWaypoint> subgoal_list;
        bool is_finish = false;
        int waypoint_num = 0;
        int waypoint_dim = 0;
        int current_stage = 0;

        // default
        string mode = "null";
        bool script_enable = false;
        float distance_margin = 0.08; 
        float heading_margin = 0.01; 
        // default
        

    public:
        bt::Action action;
        Waypoint();
        void getParam();
        void loadWaypoint();
        void scriptTranslate(float **script_waypoint);

        void transShift(float *waypoint);
        void transRotate(float *waypoint);
        void transHeading(float *waypoint);
        void transPose(float *waypoint);

        void stateCallback(const std_msgs::Bool::ConstPtr& msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void publishGoal(bool init);

        void actionSet(int state);
};

Waypoint :: Waypoint() : action("waypoint planner cruise"){
    pub_subgoal = n.advertise<drone_navigation::droneWaypoint>("waypoint_planner/drone_waypoint", 10);
    sub_state = n.subscribe<std_msgs::Bool>("navigation_manager/is_finish", 1, &Waypoint::stateCallback, this);
    sub_goal = n.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, &Waypoint::goalCallback, this);
}

void Waypoint :: stateCallback(const std_msgs::Bool::ConstPtr& msg){
    is_finish = msg->data;
    return;
}

void Waypoint :: goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(~script_enable){
        subgoal_list.clear();
        drone_navigation::droneWaypoint p;
        p.pose.position.x = msg->pose.position.x;
        p.pose.position.y = msg->pose.position.y;
        p.pose.position.z = msg->pose.position.z;
        p.pose.orientation.x = msg->pose.orientation.x;
        p.pose.orientation.y = msg->pose.orientation.y;
        p.pose.orientation.z = msg->pose.orientation.z;
        p.pose.orientation.w = msg->pose.orientation.w;
        p.distance_margin = distance_margin;
        p.heading_margin = heading_margin;
        if(mode == (string)"waypoint_shifting"){
            p.pose_enable = true;
            p.heading_enable = false;
            subgoal_list.push_back(p);
        }else if(mode == (string)"waypoint_rotate"){
            p.pose_enable = false;
            p.heading_enable = true;
            subgoal_list.push_back(p);
        }else if(mode == (string)"waypoint_heading"){
            p.pose_enable = false;
            p.heading_enable = true;
            subgoal_list.push_back(p);
            p.pose_enable = true;
            p.heading_enable = false;
            subgoal_list.push_back(p);
        }else if(mode == (string)"waypoint_pose"){
            p.pose_enable = true;
            p.heading_enable = true;
            subgoal_list.push_back(p);
        }
    }
    return;
}

void Waypoint :: publishGoal(bool init){
    if(subgoal_list.size() == 0){
        cout << "subgoal list is empty" << endl;
        return;
    }
    if(init){
        cout << "debug: initialize..." << endl;
        drone_navigation::droneWaypoint pub_subgoal_msg;
        pub_subgoal_msg = subgoal_list.front();
        pub_subgoal.publish(pub_subgoal_msg);
    }else{
        if(is_finish){
            is_finish = false;
            cout << "debug: recieve result, pop" << endl;
            drone_navigation::droneWaypoint pub_subgoal_msg;
            pub_subgoal_msg = subgoal_list.front();
            subgoal_list.pop_front();
            pub_subgoal.publish(pub_subgoal_msg); 
        }
    }
    cout << "list size = " << subgoal_list.size() << endl; 
    return;
}

void Waypoint :: transShift(float *waypoint){
    drone_navigation::droneWaypoint p;
    
    p.pose.position.x = waypoint[0];
    p.pose.position.y = waypoint[1];
    p.pose.position.z = waypoint[2];
    
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    
    p.pose_enable = true;
    p.heading_enable = false;
    p.distance_margin = distance_margin;
    p.heading_margin = heading_margin;
    subgoal_list.push_back(p);
    return;
}

void Waypoint :: transRotate(float *waypoint){
    drone_navigation::droneWaypoint p;
    
    p.pose.position.x = waypoint[0];
    p.pose.position.y = waypoint[1];
    p.pose.position.z = waypoint[2];
    if(waypoint_dim == 3){
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 1;
    }else if(waypoint_dim == 4){
        tf::Quaternion q_new;
        q_new.setRPY(0, 0, waypoint[3]);
        q_new = q_new.normalize();
        p.pose.orientation.x = q_new.getX();
        p.pose.orientation.y = q_new.getY();
        p.pose.orientation.z = q_new.getZ();
        p.pose.orientation.w = q_new.getW();
    }
    p.pose_enable = false;
    p.heading_enable = true;
    p.distance_margin = distance_margin;
    p.heading_margin = heading_margin;
    subgoal_list.push_back(p);
    return;
}

void Waypoint :: transHeading(float *waypoint){
    transRotate(waypoint);
    transShift(waypoint);
    return;
}

void Waypoint :: transPose(float *waypoint){
    drone_navigation::droneWaypoint p;
    
    p.pose.position.x = waypoint[0];
    p.pose.position.y = waypoint[1];
    p.pose.position.z = waypoint[2];
    if(waypoint_dim == 3){
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 1;
    }else if(waypoint_dim == 4){
        tf::Quaternion q_new;
        q_new.setRPY(0, 0, waypoint[3]);
        q_new = q_new.normalize();
        p.pose.orientation.x = q_new.getX();
        p.pose.orientation.y = q_new.getY();
        p.pose.orientation.z = q_new.getZ();
        p.pose.orientation.w = q_new.getW();
    }
    p.pose_enable = true;
    p.heading_enable = true;
    p.distance_margin = distance_margin;
    p.heading_margin = heading_margin;
    subgoal_list.push_back(p);
    return;
}

void Waypoint :: scriptTranslate(float **script_waypoint){
    for(int i = 0; i < waypoint_num; i++){
        if(mode == (string)"waypoint_shifting"){
            transShift(script_waypoint[i]);
        }else if(mode == (string)"waypoint_rotate"){
            transRotate(script_waypoint[i]);
        }else if(mode == (string)"waypoint_heading"){
            transHeading(script_waypoint[i]);
        }else if(mode == (string)"waypoint_pose"){
            transPose(script_waypoint[i]);
        }
    }
    return;
}

void Waypoint :: loadWaypoint(){
    waypoint_list = new float *[waypoint_num];
    for (int i = 0; i < waypoint_num; i++){ waypoint_list[i] = new float[waypoint_dim]; }
    for (int i = 0; i < waypoint_num; i++){
        for (int j = 0; j < waypoint_dim; j++){
            try{
                std::ostringstream ostr;
                ostr << xml_waypoint[i][j];
                std::istringstream istr(ostr.str());
                istr >> waypoint_list[i][j];
            }
            catch(...){
                throw;
            }
        }
    }
    return;
}

void Waypoint :: getParam(){
    string node_ns = ros::this_node::getName();
    cout << node_ns << endl;
    n.getParam("/" + node_ns + "/waypoint", xml_waypoint);
    n.getParam("/" + node_ns + "/distance_margin", distance_margin);
    n.getParam("/" + node_ns + "/heading_margin", heading_margin);
    n.getParam("/" + node_ns + "/script_enable", script_enable);
    n.getParam("/" + node_ns + "/mode", mode);
    waypoint_num = xml_waypoint.size();
    waypoint_dim = xml_waypoint[0].size();

    cout << "test: " << endl;
    cout << distance_margin << endl;
    cout << heading_margin << endl;
    cout << script_enable << endl;
    cout << mode << endl;

    if(script_enable){
        loadWaypoint();
        scriptTranslate(waypoint_list);
    }
    return;
}

void Waypoint :: actionSet(int state){
    switch(state){
        case 1:
            action.set_success();
            break;
        case 0:
            action.set_running();
            break;
        case -1:
            action.set_failure();
            break;
    }
    action.publish();
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "waypoint_offboard");

    Waypoint cruise;
    cruise.getParam();

    while(ros::ok()){
        
        if(cruise.action.is_active() && cruise.action.active_has_changed()){
            ROS_INFO("Action: Start");
        }
        else if(cruise.action.active_has_changed() && !(cruise.action.is_active())){
            ROS_INFO("Action: Done");
            cruise.actionSet(1);
        }
        else if(cruise.action.is_active()){
            cruise.actionSet(0);
            cruise.publishGoal(false);
        }
        else{
            cruise.publishGoal(true);
        }
        ros::spinOnce();
    }
    return 0;
}