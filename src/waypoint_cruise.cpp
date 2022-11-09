#include <iostream>
#include <string> 
#include <list>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <behavior_tree/behavior_tree.h>
#include <drone_navigation/droneWaypoint.h>

#define PI 3.14159265

using namespace std; 

string appendString(const string &s_body, const string &s_suffix){
    std::string origin = s_body;
    std::string later = s_suffix;
    origin.append(later);
    return origin;
}


class Waypoint{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_subgoal;
        ros::Publisher pub_explore_status;
        ros::Subscriber sub_state;
        ros::Subscriber sub_goal;
        

        drone_navigation::droneWaypoint pub_current_subgoal;

        XmlRpc::XmlRpcValue xml_waypoint;

        float **waypoint_list;

        
        list<drone_navigation::droneWaypoint> subgoal_list;
        list<drone_navigation::droneWaypoint> backup_list;
        bool navigation_manager_status = false;
        bool waypoint_running = false;
        int waypoint_planner_action = -1;
        int waypoint_num = 0;
        int waypoint_dim = 0;
        int current_waypoint = 1;


        // default
        string mode = "null";
        string origin = "map";
        string planner_name = "none";
        unsigned int planner_seq = 0;
        bool script_enable = false;
        float distance_margin = 0.08; 
        float heading_margin = 0.01; 
        float cruise_height = 9.0;
        // default
        

    public:
        bt::Action action;
        bt::Condition condition_running;
        Waypoint();
        void getParam();
        void loadWaypoint();
        void scriptTranslate(float **script_waypoint);

        void init();
        void recap();
        void structListCopy(list<drone_navigation::droneWaypoint> &origin, list<drone_navigation::droneWaypoint> &copy);

        void transShift(float *waypoint);
        void transRotate(float *waypoint);
        void transHeading(float *waypoint);
        void transPose(float *waypoint);

        void stateCallback(const std_msgs::Bool::ConstPtr& msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void publishGoal();
        

        void actionSet(int state);
        void conditionRunningSet(bool state);
        void setLastCondition();
        void prolongEvalState(bool state);
};

Waypoint :: Waypoint() : action(ros::this_node::getName()), 
                        condition_running(appendString(ros::this_node::getName(), (string)"_running")){
    pub_subgoal = n.advertise<drone_navigation::droneWaypoint>("waypoint_planner/drone_waypoint", 10);
    pub_explore_status = n.advertise<std_msgs::Bool>("waypoint_planner/finish_explore", 10);
    sub_state = n.subscribe<std_msgs::Bool>("navigation_manager/is_finish", 1, &Waypoint::stateCallback, this);
    sub_goal = n.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, &Waypoint::goalCallback, this);
}

void Waypoint :: stateCallback(const std_msgs::Bool::ConstPtr& msg){
    if(waypoint_running){
        navigation_manager_status = msg->data;
    }
    return;
}

void Waypoint :: goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(!script_enable){
        // init();
        // cout << "recieved goal" << endl;
        subgoal_list.clear();
        drone_navigation::droneWaypoint p;
        p.pose.position.x = msg->pose.position.x;
        p.pose.position.y = msg->pose.position.y;
        p.pose.position.z = msg->pose.position.z + cruise_height;
        p.pose.orientation.x = msg->pose.orientation.x;
        p.pose.orientation.y = msg->pose.orientation.y;
        p.pose.orientation.z = msg->pose.orientation.z;
        p.pose.orientation.w = msg->pose.orientation.w;
        p.distance_margin = distance_margin;
        p.heading_margin = heading_margin;
        p.origin = origin;
        p.planner_seq = planner_seq;
        p.planner_name = planner_name;
        planner_seq++;
        if(mode == (string)"waypoint_shifting"){
            p.pose_enable = true;
            p.rotate_enable = false;
            p.heading_enable = false;
            // p.last_waypoint = false;
            // subgoal_list.push_back(p);
            p.last_waypoint = true;
            subgoal_list.push_back(p);
        }else if(mode == (string)"waypoint_rotate"){
            p.pose_enable = false;
            p.rotate_enable = true;
            p.heading_enable = false;
            // p.last_waypoint = false;
            // subgoal_list.push_back(p);
            p.last_waypoint = true;
            subgoal_list.push_back(p);
        }else if(mode == (string)"waypoint_heading"){
            p.pose_enable = false;
            p.rotate_enable = true;
            p.heading_enable = true;
            p.last_waypoint = false;
            // subgoal_list.push_back(p);
            subgoal_list.push_back(p);
            p.pose_enable = true;
            p.rotate_enable = false;
            p.heading_enable = true;
            p.last_waypoint = true;
            subgoal_list.push_back(p);
        }else if(mode == (string)"waypoint_pose"){
            p.pose_enable = true;
            p.rotate_enable = true;
            p.heading_enable = false;
            // p.last_waypoint = false;
            // subgoal_list.push_back(p);
            p.last_waypoint = true;
            subgoal_list.push_back(p);
        }
    }
    return;
}

void Waypoint :: structListCopy(list<drone_navigation::droneWaypoint> &origin, list<drone_navigation::droneWaypoint> &copy){
    copy.assign(origin.begin(), origin.end());
    return;
}

void Waypoint :: init(){
    waypoint_running = true;
    return;
}

void Waypoint :: recap(){
    structListCopy(backup_list, subgoal_list);
    // cout << "recap finish, size: " << subgoal_list.size() << endl;
    planner_seq++;
    waypoint_running = false;
    return;
}

void Waypoint :: setLastCondition(){
    ros::Rate rate(10);
    // if(waypoint_planner_action == 1){
    //     waypoint_running = false;
    // }
    waypoint_running = false;
    conditionRunningSet(waypoint_running);
    rate.sleep();
    return;
}

void Waypoint :: prolongEvalState(bool state){
    ros::Rate rate(10);
    for(int i = 0; i < 15; i++){
        conditionRunningSet(state);
        rate.sleep();
    }
    return;
}

void Waypoint :: publishGoal(){
    ros::Rate rate(10);

    if(subgoal_list.size() > 0){ 
        pub_current_subgoal = subgoal_list.front();
        waypoint_running = true;
    }
    pub_subgoal.publish(pub_current_subgoal);
    
    
    if(navigation_manager_status){
        navigation_manager_status = false;
        if(subgoal_list.size() > 0){ 
            // cout << "pop happen" << endl;
            subgoal_list.pop_front();
        }
    }

    if(subgoal_list.size() == 0){
        // cout << "set running: false" << endl;
        waypoint_running = false;
    }

    cout << "L: " << subgoal_list.size();
    cout << " S: " << waypoint_running << endl;
    // if(waypoint_running == false){
    //     prolongEvalState(true);
    // }

    conditionRunningSet(waypoint_running);

    rate.sleep();
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
    p.rotate_enable = false;
    p.heading_enable = false;
    if(current_waypoint == waypoint_num){
        p.last_waypoint = true;
    }else{
        p.last_waypoint = false;
    }
    p.distance_margin = distance_margin;
    p.heading_margin = heading_margin;
    p.origin = origin;
    p.planner_seq = planner_seq;
    p.planner_name = planner_name;
    subgoal_list.push_back(p);

    current_waypoint++;
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
        q_new.setRPY(0, 0, (waypoint[3] * PI / 180));
        q_new = q_new.normalize();
        p.pose.orientation.x = q_new.getX();
        p.pose.orientation.y = q_new.getY();
        p.pose.orientation.z = q_new.getZ();
        p.pose.orientation.w = q_new.getW();
    }
    p.pose_enable = false;
    p.rotate_enable = true;
    p.heading_enable = false;
    if(current_waypoint == waypoint_num){
        p.last_waypoint = true;
    }else{
        p.last_waypoint = false;
    }
    p.distance_margin = distance_margin;
    p.heading_margin = heading_margin;
    p.origin = origin;
    p.planner_seq = planner_seq;
    p.planner_name = planner_name;
    subgoal_list.push_back(p);

    current_waypoint++;
    return;
}

void Waypoint :: transHeading(float *waypoint){
    drone_navigation::droneWaypoint p;
    p.pose.position.x = waypoint[0];
    p.pose.position.y = waypoint[1];
    p.pose.position.z = waypoint[2];
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    p.distance_margin = distance_margin;
    p.heading_margin = heading_margin;
    p.origin = origin;
    p.planner_seq = planner_seq;
    p.planner_name = planner_name;
    // rotate first
    p.pose_enable = false;
    p.rotate_enable = true;
    p.heading_enable = true;
    subgoal_list.push_back(p);

    // then shift
    p.pose_enable = true;
    p.rotate_enable = false;
    p.heading_enable = false;
    if(current_waypoint == waypoint_num){
        p.last_waypoint = true;
    }else{
        p.last_waypoint = false;
    }
    subgoal_list.push_back(p);

    current_waypoint++;
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
        q_new.setRPY(0, 0, (waypoint[3] * PI / 180));
        q_new = q_new.normalize();
        p.pose.orientation.x = q_new.getX();
        p.pose.orientation.y = q_new.getY();
        p.pose.orientation.z = q_new.getZ();
        p.pose.orientation.w = q_new.getW();
    }
    p.pose_enable = true;
    p.rotate_enable = true;
    p.heading_enable = false;
    if(current_waypoint == waypoint_num){
        p.last_waypoint = true;
    }else{
        p.last_waypoint = false;
    }
    p.distance_margin = distance_margin;
    p.heading_margin = heading_margin;
    p.origin = origin;
    p.planner_seq = planner_seq;
    p.planner_name = planner_name;
    subgoal_list.push_back(p);

    current_waypoint++;
    return;
}

void Waypoint :: scriptTranslate(float **script_waypoint){
    for(int i = 0; i < waypoint_num; i++){
        for(int j = 0; j < waypoint_dim; j++){
            cout << script_waypoint[i][j] << " " ;
        }   
        cout << endl;
    }

    // if(mode == (string)"waypoint_shifting"){
    //     transShift(script_waypoint[0]);
    // }else if(mode == (string)"waypoint_rotate"){
    //     transRotate(script_waypoint[0]);
    // }else if(mode == (string)"waypoint_heading"){
    //     transShift(script_waypoint[0]);
    // }else if(mode == (string)"waypoint_pose"){
    //     transPose(script_waypoint[0]);
    // }

    for(int i = 0; i < waypoint_num; i++){
        if(mode == (string)"waypoint_shifting"){
            transShift(script_waypoint[i]);
        }else if(mode == (string)"waypoint_rotate"){
            transRotate(script_waypoint[i]);
        }else if(mode == (string)"waypoint_heading"){
            if(i == 0){ 
                transShift(script_waypoint[i]);
                continue; 
            }
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
    // cout << node_ns << endl;
    n.getParam("/" + node_ns + "/waypoint", xml_waypoint);
    n.getParam("/" + node_ns + "/distance_margin", distance_margin);
    n.getParam("/" + node_ns + "/heading_margin", heading_margin);
    n.getParam("/" + node_ns + "/cruise_height", cruise_height);
    n.getParam("/" + node_ns + "/script_enable", script_enable);
    n.getParam("/" + node_ns + "/mode", mode);
    n.getParam("/" + node_ns + "/origin", origin);
    waypoint_num = xml_waypoint.size();
    waypoint_dim = xml_waypoint[0].size();
    planner_name = ros::this_node::getName();
    // debug
    // cout << "test: " << endl;
    // cout << distance_margin << endl;
    // cout << heading_margin << endl;
    // cout << script_enable << endl;
    // cout << mode << endl;
    // cout << origin << endl;
    // debug

    if(script_enable){
        loadWaypoint();
        scriptTranslate(waypoint_list);
        structListCopy(subgoal_list, backup_list);
        // debug
        cout << "size: " << subgoal_list.size() << endl;
        drone_navigation::droneWaypoint test;
        int i = 1;
        while(subgoal_list.size() > 0){
            test = subgoal_list.front();
            subgoal_list.pop_front();
            cout << "index: " << i << endl;
            cout << " x: " << test.pose.position.x;
            cout << " y: " << test.pose.position.y;
            cout << " z: " << test.pose.position.z << endl;
            if(test.pose_enable){   cout << "enable pose" << endl;}
            if(test.rotate_enable){ cout << "enable rotate" << endl;}
            if(test.heading_enable){cout << "enable heading" << endl;}
            i++;
        }
        structListCopy(backup_list, subgoal_list);
        // debug
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
    waypoint_planner_action = state;
    action.publish();
    return;
}

void Waypoint :: conditionRunningSet(bool state){
    condition_running.set(state);
    condition_running.publish();
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "waypoint_offboard");
    
    Waypoint cruise;
    cruise.getParam();

    while(ros::ok()){
        if(cruise.action.is_active() && cruise.action.active_has_changed()){
            // ROS_INFO("Action: Start");
            cruise.init();
        }
        else if(cruise.action.active_has_changed() && !(cruise.action.is_active())){
            // ROS_INFO("Action: Done");
            cruise.recap();
            cruise.actionSet(1);
        }
        else if(cruise.action.is_active()){
            cruise.actionSet(0);
            cruise.publishGoal();
        }else{
            cruise.setLastCondition();
        }
        ros::spinOnce();
    }
    return 0;
}