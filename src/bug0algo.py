#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
#Services require their own type of messages known as Service Messages
#SetBool is predefined ServiceMessage which has a variable called data of type bool
from std_srvs.srv import SetBool

region={
    "eright": 0,
    "right" : 0,
    "center": 0,
    "left" : 0,
    "eleft": 0
    }

srv_wall_follow = None
srv_go_to_pos = None
current_pos = Point()
desired_position = Point()
desired_position.x = rospy.get_param('dest_pos_x')
desired_position.y = rospy.get_param('dest_pos_y')
desired_position.z = 0
yaw = 0
err_dist = 0
off_angle = 0
state = 0

def change_state(n):
    global state , srv_go_to_pos , srv_wall_follow
    state = n
    rospy.loginfo("Changed to state [{}]".format(n))
    if( n==0 ):
        resp = srv_go_to_pos(True)
        resp = srv_wall_follow(False)
    elif( n==1 ):
        resp = srv_go_to_pos(False)
        resp = srv_wall_follow(True)
    else:
        rospy.loginfo("Invalid State")

def calculate_err_dist():
    global err_dist , desired_position , current_pos
    err_dist = math.sqrt(math.pow(desired_position.y - current_pos.y , 2 ) + math.pow(desired_position.x - current_pos.x , 2))
    return err_dist

def calculate_off_angle():
    global yaw , desired_position , current_pos , off_angle
    off_angle = math.atan2(desired_position.y - current_pos.y , desired_position.x - current_pos.x )
    off_angle = off_angle - yaw
    #this is called normalization. this makes the bot orient towards the goal in the least posible rotation
    if (math.fabs(off_angle) > math.pi):
        off_angle = off_angle - ((2 * math.pi * off_angle)/off_angle)
    return off_angle



def clbk_odom(msg):
    global current_pos , yaw
    current_pos = msg.pose.pose.position
    Quarternion = ([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    ])
    euler_form = transformations.euler_from_quaternion(Quarternion)
    yaw = euler_form[2]



def clbk_laser(msg):
    global region
    region = {
        "eright" : min(min(msg.ranges[0:143]) , 2 ),
        "right" :  min(min(msg.ranges[144:287]) , 2 ),
        "center" : min(min(msg.ranges[288:431]) , 2 ),
        "left" :   min(min(msg.ranges[432:575]) , 2 ),
        "eleft" : min(min(msg.ranges[576:719]) , 2 )
    }

#state 0 = go_to_point
#state 1 = wall_follow
def main():
    global srv_wall_follow , srv_go_to_pos
    rospy.init_node("wallfollowplusgotopos")
    sub_laser = rospy.Subscriber("/m2wr/laser/scan" , LaserScan , clbk_laser)
    sub_odom = rospy.Subscriber("/odom" , Odometry, clbk_odom)

    srv_wall_follow = rospy.ServiceProxy("/serv_wall_follow" , SetBool)
    srv_go_to_pos = rospy.ServiceProxy("/serv_go_to_pos", SetBool)

    #We assume that initially there are no obstacle
    change_state(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        calculate_err_dist()
        calculate_off_angle()

# Change to wall follow if there is an obstacle in front of you
        if(state==0):
            if (region["center"] < 1.5 and region["center"] > 0.15):
                change_state(1)
#Get out of wall follow if bot is in goal orientation angle sensed by laser and there is no obstacle there
#case 1 is that the offset angle is between -pi/3 to +pi/3 and the center+somepartofleft+somepartofright region of laser scan detects obstacles for -pi/3 to +pi/3 angles
#therefore if there are no obstacles we orient the bot towards the goal and go towards it , this is done by our go_to_pose script        
        if (state==1):
            if((off_angle > 0) and (math.fabs(off_angle) < (math.pi)/3) and (region["center"] >1.5 or region["left"] > 1.5 or region["right"] > 1.5)):
                print(1)
                change_state(0)
            elif(off_angle > 0 and math.fabs(off_angle) > (math.pi)/3 and math.fabs(off_angle) < abs(math.pi)/2 and region["eleft"] > 1.5 and region["left"] > 1.5):
                print(2)
                change_state(0)
            elif(off_angle < 0 and math.fabs(off_angle) > (math.pi)/3 and math.fabs(off_angle) < abs(math.pi)/2 and region["eright"] > 1.5 and region["right"] > 1.5 ):
                print(3)
                change_state(0)
        rate.sleep()
        

if __name__ == "__main__" :
    main()
