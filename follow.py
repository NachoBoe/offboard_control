#!/usr/bin/python3


import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandHome, CommandHomeRequest
from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import NavSatFix

from std_msgs.msg import String
import threading

current_state = State()
current_pose = PoseStamped()
received_pose = PoseStamped()
first_message = False

global_pose = NavSatFix()

def pose_glob_callback(msg):
    global global_pose
    global_pose = msg

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

def received_pose_cb(msg):
    global first_message 
    first_message = True
    global received_pose
    received_pose = msg

def update_waypoints():

    # Create a reciever for the pose messages
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_rate = rospy.Rate(20.0)  
    while not rospy.is_shutdown():
        received_pose_chg = received_pose
        received_pose_chg.pose.position.x += 3
        received_pose_chg.pose.position.y += 3
        received_pose_chg.pose.position.z += 2
        local_pos_pub.publish(received_pose_chg)
        local_rate.sleep()

def main():
    latitude_home = -34.91784
    longitud_home = -56.16748
    takeoff_altitude = 2 # MIS_TAKEOFF_ALT to select alttitud in auto.takeoff (default 2.5)

    rospy.init_node('square', anonymous=True)
    rate = rospy.Rate(20.0)  # 20Hz

    state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
    pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
    received_msg_pub = rospy.Subscriber('/pose_received', PoseStamped, received_pose_cb)
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, pose_glob_callback)


    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    rospy.wait_for_service("/mavros/cmd/set_home")
    set_home_client = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
    
    # Wait for MAVROS to connect
    rospy.loginfo("Starting connection...")
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    
    rospy.loginfo("Connected with px4")


    rospy.loginfo("Setting Home")

    # Set HOME
    set_home_cmd = CommandHomeRequest()
    set_home_cmd.yaw = 0
    set_home_cmd.latitude = latitude_home
    set_home_cmd.longitude = longitud_home
    set_home_cmd.altitude = global_pose.altitude*0.66216
    rospy.loginfo(global_pose)


    res = set_home_client.call(set_home_cmd)
    while (res.success == False):
        res = set_home_client.call(set_home_cmd)
        rate.sleep()
        rospy.loginfo(res.success)

    rospy.loginfo("Home set correctly")

    rospy.loginfo("Waiting for first message from other drone")
    while first_message==False:
        rate.sleep()
    rospy.loginfo("First message recieved")

    rospy.loginfo("Waiting for other drone to takeoff")
    while abs(received_pose.pose.position.z) < 1:
        rate.sleep()


    # Start sending the waypoints
    receive_thread = threading.Thread(target=update_waypoints)
    receive_thread.start()


    # ARM DRONE
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    rospy.loginfo(f'Current State is {current_state.mode }')
    rospy.loginfo("Arming Drone...")
    res = arming_client.call(arm_cmd)
    while (not current_state.armed):
        res = arming_client.call(arm_cmd)
        if not current_state.armed:
            rospy.loginfo("Unable to arm Vehicle, trying again...")
            rate.sleep()

    rospy.loginfo("Vehicle armed")


    # TAKEOFF


    takeoff_pose = PoseStamped()

    takeoff_pose.pose.position.x = 0
    takeoff_pose.pose.position.y = 0
    takeoff_pose.pose.position.z = takeoff_altitude

    # Change the mode to TAKEOFF
    takeoff_req = SetModeRequest()
    takeoff_req.custom_mode = "AUTO.TAKEOFF"

    rospy.loginfo("Setting mode to TAKEOFF...")
    res = set_mode_client.call(takeoff_req)
    while (current_state.mode != "AUTO.TAKEOFF"):
        res = set_mode_client.call(takeoff_req)
        rate.sleep()


    ## Wait until takeoff is completed
    #rospy.loginfo("Taking off to {} meters...".format(takeoff_altitude))
    #while abs(takeoff_pose.pose.position.z - current_pose.pose.position.z) > 0.1:
    #    rospy.loginfo(f'Difference of positions: {takeoff_pose.pose.position.z - current_pose.pose.position.z}')
    #    rate.sleep()

    rospy.loginfo("Take off completed")



    # Change the mode to OFFBOARD


    offboard_req = SetModeRequest()
    offboard_req.custom_mode = "OFFBOARD"

    rospy.loginfo(f'Current State is {current_state.mode }, changing to OFFBOARD')
    res = set_mode_client.call(offboard_req)

    while (current_state.mode != "OFFBOARD"):
        res = set_mode_client.call(offboard_req)
        if current_state.mode != "OFFBOARD":
            rate.sleep()
            rospy.loginfo('Unable to set OFFBOARD, trying again...')

    rospy.loginfo('OFFBOARD reached')

    rospy.loginfo('Following Target...')
    while abs(received_pose.pose.position.z) > 1:
        rate.sleep()

    # LANDING
    land_req = SetModeRequest()
    land_req.custom_mode = "AUTO.LAND"

    rospy.loginfo("Setting mode to LAND...")

    while (current_state.mode != "AUTO.LAND"):
        res = set_mode_client.call(land_req)
        rospy.loginfo(res)
        rospy.sleep(0.5)

    rospy.loginfo("LAND set")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass