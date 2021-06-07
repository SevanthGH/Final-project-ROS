#!/usr/bin/env python
import rospy
import math
import actionlib
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist          # Import the Twist message from the geometry_msgs
from sensor_msgs.msg import LaserScan       # Import the LaserScan message from the sensor #import ModelState message from gazebo_msgs
from geometry_msgs.msg import Pose,Point         # Import the Pose message from the geometry_msgs
from tf.transformations import euler_from_quaternion
from goal_publisher.msg import PointArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pow, atan2, sqrt
import pdb
import os
import operator
import collections



#global varibles declaration

final_points=[]
base_y=0
base_x=0

def amcl(pos):     # Model state function which is continously providing x,y and theta value of robot while moving
    global base_x
    global base_y
    global current_theta
    base_x = pos.pose.pose.position.x
    base_y = pos.pose.pose.position.y
    rot_q=pos.pose.pose.orientation
    (roll,pitch,current_theta)=euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])


def callback(msg):        #callback function which is continously subscribing the Goals from goal_publisher package
    global final_points
    for i in range(len(msg.goals)):
        final_points.append([msg.goals[i].x, msg.goals[i].y, msg.goals[i].z, msg.goals[i].reward])
    return(final_points)

def move(point):
    goal =MoveBaseGoal()
    goal.target_pose.header.frame_id="map"
    goal.target_pose.pose.position.x=2
    goal.target_pose.pose.position.y=1
    goal.target_pose.pose.orientation.w = 1
    return goal
    

def sort_goal_list(final_points):
    fractions = []
    my_dict = {}
    distance_list = []

    for i in range(len(final_points)):
        x = final_points[i][0]
        #print(x)
        y = final_points[i][1]
        r = final_points[i][3]
        #print(r)
        f = pow(r+x+y,2)
        #print(f)
        fractions.append(f)
        #print(my_dict)
        my_dict[f] = [x, y , r]
        #print(my_dict)



    # Sorted dictionary in Tuple Form
    intermediate_dict = sorted(my_dict.items(), key= operator.itemgetter(0), reverse=False)


    # Final Sorted Dictionary in usable form
    sorted_dict = collections.OrderedDict(intermediate_dict)
    #print(sorted_dict.values())
    #pdb.set_trace()

    final_goal_list = []
    for value in sorted_dict.values():
        final_goal_list.append(value)
    return final_goal_list



def stop_it(dest_position):   #   Stopping the robot after reaching the Goal Point
    global speed,i,print_x,print_y,print1_x,print1_y,j
    print("#######################################################################")
    print("For Goal:{}  The target position of robot is x:{} and y :{}".format(j,dest_position.x,dest_position.y))
    print("Goal:{} reached.The position of robot is x:{} and y :{}".format(j,x,y))
    print("#######################################################################")
    speed.linear.x =0
    speed.angular.z =0
    pub.publish(speed)
    rospy.sleep(2)
    print1_x=print_x[j:]
    print1_y=print_y[j:]
    j=j+1
    change_state(0)


def main():
       global print_x,pub,error_position,i,error_theta,print1_x,print1_y,final_points,msg, goal1
       rospy.init_node('final')
       rate = rospy.Rate(10)
       goals_sub = rospy.Subscriber('/goals',PointArray,callback)
       pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped , amcl)
       client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
       client.wait_for_server(rospy.Duration(1))
       rospy.sleep(2)
       final_ordered_goals_list = sort_goal_list(final_points)

       while not rospy.is_shutdown():
            result = move(final_ordered_goals_list[0])
            client.send_goal(result)
            client.wait_for_result()
            
if __name__ == '__main__':
  	main()
        rate.sleep()
