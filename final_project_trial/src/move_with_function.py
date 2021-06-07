#!/usr/bin/env python

import rospy
import operator
from math import pow, sqrt
import time
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from goal_publisher.msg import PointArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
# from gerty.msg import GertyRoboAction, GertyRoboGoal, GertyRoboFeedback, GertyRoboResult

class move():
    
    
    
    #######################  Initialise at start of code    ##########################3
    def __init__(self):
     	self.count=0
        self.flag=0
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)     #Publish cmd_vel topic
        self.vel = Twist()
        time.sleep(2)
        if self.flag == 0:
            print('making one rotation for localization')
            self.one_rotation()
            self.flag=1
        else:
            pass
        self.sub = rospy.Subscriber('/goals', PointArray, self.reach_goal)     
        print(1)
        #rospy.sleep(5)
        #self.sub_2 = rospy.Subscriber('/gazebo/model_states', ModelStates, self.check_tolerance)
        self.pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 1)
        self.robot_pos = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.robot_position)
        time.sleep(2)
        self.status = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_of_robot)
        #self.pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 1)
        #self.pub_2 = rospy.Publisher('/get_robo_stat', robot_status, queue_size = 1)
        self.rate = rospy.Rate(10)
        #self.stat = robot_status()
        #self.stat.robo_stat = "Reached"
        #self.pub_2.publish(self.stat)
        self.x_dest = 0
        self.y_dest = 0
        self.x_pos = 0
        self.y_pos = 0
        self.robostatus = 0
        self.i = 0
       
        self.temp = ''
             
        
############################# Function to traverse through goals#################
    def reach_goal(self, dest):
        self.length_of_goals = len(dest.goals)
        #print(length_of_goals)
        #for i in range(length_of_goals):
        #if self.i < 10:
        self.coord = dest.goals 
        #print (self.coord)
        self.x_dest = self.coord[self.count].x
        print ("x point is ",self.x_dest )
        self.y_dest = self.coord[self.count].y
        print ("y point is ",self.y_dest )
        print(self.x_dest,self.y_dest)
        print(self.x_pos,self.y_pos)
       # print(self.i)
        self.dest_p = MoveBaseActionGoal()
        self.dest_p.goal.target_pose.header.frame_id = "map"
        self.dest_p.goal.target_pose.pose.position.x = self.x_dest
        self.dest_p.goal.target_pose.pose.position.y = self.y_dest
        self.dest_p.goal.target_pose.pose.orientation.w = 1
        self.pub.publish(self.dest_p)
        time.sleep(1)
	self.get_status()
##############Function to peform 1 rotation to get localisation########################
    def one_rotation(self):
        self.vel.angular.z = 0.8
        self.vel.linear.x = 0.0
        self.pub_vel.publish(self.vel)
        time.sleep(20)                 ##delay of 20 secs
        self.vel.angular.z = 0.0
        self.vel.linear.x = 0.0
        self.pub_vel.publish(self.vel)
##################Position of Robot###############
    def robot_position(self,present_pos):
        self.x_pos = present_pos.pose.pose.position.x
        self.y_pos = present_pos.pose.pose.position.y
        print(self.x_pos,self.y_pos)


###############Status of Bot############################
    def status_of_robot(self, robot_status):
        self.a_status=robot_status.status_list[0].status
  
##############Set status once goal point is reached###########
    def get_status(self):
        #print("status is :",self.a_status)
        if self.a_status == 3:
        	print (self.a_status)
   		print "reached point ",self.count+1,(self.x_dest,self.y_dest)
   		time.sleep(1)                                        ### wait for 1 second
		if self.count< self.length_of_goals:
		   self.count=self.count+1                 #increment count to go to next point till all goal points are reached
		   self.a_status = 2
		else:
		    self.count=0    #start from zero when all goals reached
		    self.a_status= 2                    
	    		
if __name__=='__main__':
   try:
     rospy.init_node('move')
     move()
     rospy.spin()
   except rospy.ROSInterruptException:
        pass
