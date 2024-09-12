#!/usr/bin/env python
import rospy
from mavros_msgs.srv import SetMode, SetModeResponse
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

current_state = None
current_pose = None

def state_callback(state):
    """
    Callback function to update the current state of the UAV.
    """
    global current_state
    current_state = state

def pose_callback(pose):
    """
    Callback function to update the current pose of the UAV.
    """
    global current_pose
    current_pose = pose

def set_mode(mode):
    """
    Set the mode of the UAV using the SetMode service.
    """
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = set_mode_service(base_mode=0, custom_mode=mode)
        if response.mode_sent:
            rospy.loginfo("Mode change to {} was successful.".format(mode))
            return True
        else:
            rospy.logwarn("Mode change to {} failed.".format(mode))
            return False
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        return False

def handle_takeoff(req):
    """
    Service handler for takeoff request.
    """
    # Ensure that the UAV is connected and the state is available
    while not rospy.is_shutdown() and current_state is None:
        rospy.sleep(0.1)
    
    # Set OFFBOARD mode
    if not set_mode("OFFBOARD"):
        return SetModeResponse(success=False)
    
    # Setup target position for takeoff
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 3  # Takeoff altitude

    # Publish the target position repeatedly to ensure mode stability
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(20)

    for _ in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    rospy.loginfo("Takeoff command sent, waiting for takeoff...")

    # Continue publishing target position to maintain OFFBOARD mode
    while not rospy.is_shutdown():
        local_pos_pub.publish(pose)
        rate.sleep()

    return SetModeResponse(success=True)

def takeoff_service_client():
    """
    Service client to request takeoff.
    """
    rospy.wait_for_service('/takeoff')
    try:
        takeoff_service = rospy.ServiceProxy('/takeoff', SetMode)
        response = takeoff_service()
        if response.mode_sent:
            rospy.loginfo("Takeoff request was successful.")
        else:
            rospy.logwarn("Takeoff request failed.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def main():
    """
    Main function to initialize the node and start the service server.
    """
    rospy.init_node('mavros_takeoff_service_node', anonymous=True)
    
    # Subscribe to UAV state and position topics
    rospy.Subscriber('/mavros/state', State, state_callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    
    # Create the service server
    rospy.Service('/takeoff', SetMode, handle_takeoff)
    rospy.loginfo("Takeoff service is ready.")


    # Call the service client for takeoff
    takeoff_service_client()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
