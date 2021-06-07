#!/usr/bin/env python

import rospy
import operator
from math import pow, sqrt
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
        self.flag=0
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
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
        self.pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size = 1)
        time.sleep(2)
        self.robot_pos = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.robot_position)
        time.sleep(2)
        self.status = rospy.Subscriber('move_base/status', GoalStatusArray, self.status_of_robot)
        #self.pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size = 1)
        #self.pub_2 = rospy.Publisher('/get_robo_stat', robot_status, queue_size = 1)
        self.rate = rospy.Rate(100)
        #self.stat = robot_status()
        #self.stat.robo_stat = "Reached"
        #self.pub_2.publish(self.stat)
        self.x_dest = []
        self.y_dest = 0
        self.x_pos = 0
        self.y_pos = 0
        self.robostatus = 0
        self.i = 0
        self.temp = ''
        self.flag = 0
        self.prestposx = 0
        self.prestposy = 0
        self.count = 0
        self.points_gained = 0
        self.skipped_goals = 0
        #######################
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
        self.rnext = 0
        #######################

    def reach_goal(self, final_goal_list):
        length_of_goals = len(final_goal_list)
        #print(length_of_goals)
        #for i in range(length_of_goals):
        #if self.i < 10:
        self.x_dest = final_goal_list[self.i]
        #self.y_dest = final_goal_list[self.i]
        #print(self.x_dest,self.y_dest)
        #print(self.x_pos,self.y_pos)
        #print(self.i)
        self.dest_p = MoveBaseActionGoal()
        self.dest_p.goal.target_pose.header.frame_id = "map"
        self.dest_p.goal.target_pose.pose.position.x = self.x_dest[0]
        self.dest_p.goal.target_pose.pose.position.y = self.x_dest[1]
        self.dest_p.goal.target_pose.pose.orientation.w = 1
        self.pub.publish(self.dest_p)
        print(sqrt((self.x_dest[0]-self.prestposx)**2 + (self.x_dest[1]-self.prestposy)**2))
        if sqrt((self.x_dest[0]-self.x_pos)**2 + (self.x_dest[1]-self.y_pos)**2) < 0.2 or self.goal_reached == 3:
            #self.points_gained = self.points_gained + self.final_goal_list[2]
            print('Goal reached rewards gained')
            #print(self.points_gained)
            self.i = self.i + 1
            self.count = 0
            self.prestposx = self.x_pos
            self.prestposy = self.x_pos
            self.skipped_goals = 0
            rospy.sleep(0.1)
        if sqrt((self.x_dest[0]-self.prestposx)**2 + (self.x_dest[1]-self.prestposy)**2) > 0.2:
            self.count = self.count + 1
            rospy.sleep(5)
            print('Waiting time for 60 secs for robot to reach goal', self.count*10)
            if self.count >= 7:
                print('goal skipped')
                self.count = 0
                #self.skipped_goals = self.skipped_goals + 1
                #if self.skipped_goals == 2:
                #    one_rotation()
                #    self.i = self.i - 1
                #skipped_goalx[a] = self.x_dest
                #skipped_goaly[a] = self.x_dest
                #a = a + 1
                self.i = self.i + 1

    def goals(self,target):
        for i in range(len(target.goals)):
              self.final_points.append([target.goals[i].x, target.goals[i].y, target.goals[i].z, target.goals[i].reward])
        self.sorting(self.final_points)


    def sorting(self,final_points):
        for self.j in range(len(self.final_points)):
              self.x = self.final_points[self.j][0]
              self.y = self.final_points[self.j][1]
              self.r = self.final_points[self.j][3]
              #if self.j != len(self.final_points):
                #  self.rnext = self.final_points[self.j+1][3]
              self.f = (self.r) ##key value
              self.fractions.append(self.f)
              self.my_dict[self.f] = [self.x, self.y , self.r]
        self.intermediate_dict = sorted(self.my_dict.items(), key= operator.itemgetter(0), reverse=False)
        self.sorted_dict = collections.OrderedDict(self.intermediate_dict)

    	for value in self.sorted_dict.values():
    	    self.final_goal_list.append(value)
	    self.reach_goal(self.final_goal_list)

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
        #print(self.x_pos,self.y_pos)

    def status_of_robot(self, robot_status):
        self.goal_reached = robot_status.status_list[0].status
        print(self.goal_reached)

if __name__=='__main__':
    rospy.init_node('move')
    move()
    rospy.spin()
