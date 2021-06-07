#!/usr/bin/env python

import rospy
import operator
import pdb
from math import pow, atan2, sqrt
import time
import operator
import collections
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from goal_publisher.msg import PointArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
# from gerty.msg import GertyRoboAction, GertyRoboGoal, GertyRoboFeedback, GertyRoboResult

class move():

    def __init__(self):
        self.sub = rospy.Subscriber('/goals', PointArray, self.goals)
	self.x = 0
        self.y = 0
        self.r =0
        self.f =0
        self.j = 0
        self.final_points=[]
        self.final_goal_list=[]
        self.sorted_dict=[]
        self.intermediate_dict=[] 
	self.fractions=[]
	self.dest=[]  
	self.my_dict = {}

    
    def reach_goal(self,final_goal_list):
	#print(self.final_goal_list)
        self.dest = self.final_goal_list[0]
        print(self.dest[1])
        self.dest_p = MoveBaseActionGoal()
        self.dest_p.goal.target_pose.header.frame_id = "map"
        self.dest_p.goal.target_pose.pose.position.x = self.dest[0]
        self.dest_p.goal.target_pose.pose.position.y = self.dest[1]
        self.dest_p.goal.target_pose.pose.orientation.w = 1
        #self.pub.publish(self.dest_p)
        #if sqrt((self.x_dest-self.x_pos)**2 + (self.y_dest-self.y_pos)**2) < 0.2 or self.goal_reached == 3:
         #   self.i = self.i + 1
         #   rospy.sleep(1)
	
    def goals(self,target):
        for i in range(len(target.goals)):
              self.final_points.append([target.goals[i].x, target.goals[i].y, target.goals[i].z, target.goals[i].reward])
        self.sorting(self.final_points)
  

    def sorting(self,final_points):
        for self.j in range(len(self.final_points)):
              self.x = self.final_points[self.j][0]
              self.y = self.final_points[self.j][1]
              self.r = self.final_points[self.j][3]
              self.f = pow(self.r+self.x+self.y,2)
              self.fractions.append(self.f)
              self.my_dict[self.f] = [self.x, self.y , self.r] 
        self.intermediate_dict = sorted(self.my_dict.items(), key= operator.itemgetter(0), reverse=False)
        self.sorted_dict = collections.OrderedDict(self.intermediate_dict)
        
    	for value in self.sorted_dict.values():
    	    self.final_goal_list.append(value)
	self.reach_goal(self.final_goal_list)
        

if __name__=='__main__':
    rospy.init_node('move')
    move()
    rospy.spin()
