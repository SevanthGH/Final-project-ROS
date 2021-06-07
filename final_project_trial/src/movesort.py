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

    def __init__(self):
        self.flag=0
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel = Twist()
        time.sleep(2)
        if self.flag == 0:
            print('making one rotation for localization')
            self.one_rotation()
            self.flag=1
        else:
            pass
        self.sub = rospy.Subscriber('/goals', PointArray, self.goals)
        #print(1)
        #rospy.sleep(5)
        #self.sub_2 = rospy.Subscriber('/gazebo/model_states', ModelStates, self.check_tolerance)
        self.pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 1)
        self.robot_pos = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.robot_position)
        time.sleep(2)
        self.status = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_of_robot)
        #self.pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 1)
        #self.pub_2 = rospy.Publisher('/get_robo_stat', robot_status, queue_size = 1)
        self.rate = rospy.Rate(1000)
        #self.stat = robot_status()
        #self.stat.robo_stat = "Reached"
        #self.pub_2.publish(self.stat)
        self.x_dest = 0
        self.y_dest = 0
        self.x_pos = 0
        self.y_pos = 0
        self.robostatus = 0
        self.i = 0
        self.j = 0
        self.temp = ''
        self.flag = 0  
        self.final_points=[]
        self.final_goal_list=[]
        self.sorted_dict=[]
        self.intermediate_dict=[] 
	self.dest=[]  


    def reach_goal(self,final_goal_list):
        self.dest = dest.final_goal_list[self.i]
        print(self.i)
        self.dest_p = MoveBaseActionGoal()
        self.dest_p.goal.target_pose.header.frame_id = "map"
        self.dest_p.goal.target_pose.pose.position.x = self.dest[0]
        self.dest_p.goal.target_pose.pose.position.y = self.dest[1]
        self.dest_p.goal.target_pose.pose.orientation.w = 1
        self.pub.publish(self.dest_p)
        if sqrt((self.x_dest-self.x_pos)**2 + (self.y_dest-self.y_pos)**2) < 0.2 or self.goal_reached == 3:
            self.i = self.i + 1
            rospy.sleep(1)

    def goals(self,target):
        for i in range(len(target.goals)):
              self.final_points.append([target.goals[i].x, target.goals[i].y, target.goals[i].z, target.goals[i].reward])
        self.reach_goal(final_points)


    def sorting(self,final_points)
        for self.j in range(len(self.final_points)):
              self.x = self.final_points[self.j][0]
              self.y = self.final_points[self.j][1]
              self.r = self.final_points[self.j][3]
              self.f = pow(r+x+y,2)
              self.fractions.append(self.f)
              self.my_dict[f] = [x, y , r] ##[100:[1,1,8],i] 10,
        self.intermediate_dict = sorted(self.my_dict.items(), key= operator.itemgetter(0), reverse=False)
        self.sorted_dict = collections.OrderedDict(self.intermediate_dict)
        
    	for value in self.sorted_dict.values():
    	    self.final_goal_list.append(value) #[[1,1,8],[2,2,9]
        self.reach_goal(final_goal_list)
   

    def one_rotation(self):
        self.vel.angular.z = 0.8
        self.vel.linear.x = 0.0
        self.pub_vel.publish(self.vel)
        time.sleep(20)
        self.vel.angular.z = 0.0
        self.vel.linear.x = 0.0
        self.pub_vel.publish(self.vel)

    def robot_position(self,present_pos):
        self.x_pos = present_pos.pose.pose.position.x
        self.y_pos = present_pos.pose.pose.position.y
        print(self.x_pos,self.y_pos)

    def status_of_robot(self, robot_status):
        self.goal_reached = robot_status.status_list[0].status
        print(self.goal_reached)

if __name__=='__main__':
    rospy.init_node('move')
    move()
    rospy.spin()
