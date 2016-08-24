#!/usr/bin/env python

#http://docs.ros.org/api/geometry_msgs/html/index-msg.html
#http://docs.ros.org/api/mavros_msgs/html/index-msg.html


import rospy #http://wiki.ros.org/rospy
import sys
import time
from geometry_msgs.msg import PoseStamped #http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
from geometry_msgs.msg import Pose #http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html
from mavros_msgs.msg import State #http://docs.ros.org/api/mavros_msgs/html/msg/State.html



def state_update(data):
    # this is a callback function: it is called when /mavros/state topic updates
    # the data that is passed to this function is just a State message
    # http://docs.ros.org/api/mavros_msgs/html/msg/State.html
    # which has the following parameters:
    # std_msgs/Header header
    # bool connected
    # bool armed
    # bool guided
    # string mode
    #we're accesing those parameters and storing them in global variables
    global isConnected, isArmed, mode
    isConnected = data.connected
    isArmed = data.armed
    mode = data.mode
#    

def gh_setpoint_update(data):
    # same thing here, except the message type is different- it's a Pose
    # http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html
    # exactly the same way, we're accessing the parameters of the Point message
    # which is a part of Pose
    # http://docs.ros.org/api/geometry_msgs/html/msg/Point.html
    print 'gh_setpoint_update'
    global targetX,targetY,targetZ

    targetX = float(data.position.x)
    targetY = float(data.position.y)
    targetZ = float(data.position.z)
    rospy.logwarn( 'INFO: '+'Received new destination!'+'\n'+str(targetX)+', '+str(targetY)+', '+str(targetZ) )
#

def publish_target_pose():
    
    global targetX,targetY,targetZ,loop_count
    # again, http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
    # the PoseStamped message consists of multiple other "inserted" messages:
    # std_msgs/Header - http://docs.ros.org/api/std_msgs/html/msg/Header.html
	# with "seq" parameter of type uint32
	# "time" parameter of type time
	# "frame_id" parameter of type string
    # geometry_msgs/Pose - http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html
    	# in Pose there are:
	# geometry_msgs/Point - position - http://docs.ros.org/api/geometry_msgs/html/msg/Point.html
	# geometry_msgs/Quaternion  - orientation - http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html
    #and so on
    
    setpoint_msg.header.stamp = rospy.Time.now()
    setpoint_msg.header.seq = loop_count
    setpoint_msg.header.frame_id = 'fcu'
    
    setpoint_msg.pose.position.x = targetX
    setpoint_msg.pose.position.y = targetY
    setpoint_msg.pose.position.z = targetZ
    
    setpoint_msg.pose.orientation.x = 0
    setpoint_msg.pose.orientation.y = 0
    setpoint_msg.pose.orientation.z = 0
    setpoint_msg.pose.orientation.w = 1
    
    # we publish the message
    # the topic that we publish to is specified in the declaration of the publisher object below 	
    setpoint_pub.publish(setpoint_msg)
#
def waitForFCUConnection():

    rospy.logwarn("Waiting for FCU connection...")
    while not rospy.is_shutdown():
        if isConnected:
            rospy.logwarn("FCU is connected...")
            return True
        rate.sleep()
    rospy.logwarn("ROS is shutdown")
    return False
#

rospy.init_node('commander_node')

# when /mavros/state topic updates, the state_update callback function is called
# the State message object is passed to state_update
rospy.Subscriber('/mavros/state', State, state_update) 
rospy.Subscriber('/gh/targetpose', Pose, gh_setpoint_update) #when /gh/targetpose updates, gh_setpoint_update is called

setpoint_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=2) #creates a publisher object

setpoint_msg = PoseStamped() 
#creates a message object of type pose stamped #http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html

rate = rospy.Rate(20) #the rate will take care of updating the publisher at a specified speed - 20hz in this case
loop_count = 0
targetX,targetY,targetZ = 0,0,0
isConnected = 0
isArmed = 0
mode = None

waitForFCUConnection() #Wait for the autopilot to connect

#### Main loop ###
while not rospy.is_shutdown():

    publish_target_pose()
          
    loop_count+=1  
    rate.sleep()
        
        
        
        
        
