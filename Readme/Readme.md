#####################################################################

     Final Project - Moving Turtle Boot for different goals in Gazebo

#####################################################################

Team Name: Furious_Robot Developers

Project Group Members: Kiran Siddapura Onkaraiah(@ks-192116)
	               Santosh Basavraj Tavanshi(@st-192119)
		       Sevanth Ganesha Hucchangidurga(@sh-192094 )

Degree: Masters in Electrical Engineering and Embedded systems

Semester: Second

Date:20-07-2020

#####################################################################

Short Description:

In this project we are running the robot in a built-in environment which has many obstacles and even other three robot are running simultaneously.The  main aim  is to make robot run in this environment and acheive goal point without colliding to obstacles and other robots too.For acheiving this we are using two main topic which is used for SLAM those are  move_base topic  and amcl topic.
As a project management we have created our workpackages individually  among the teammates as below 
1. Kiran :
2. Sevanth:
3. Santosh:

####################################################################

Initial Steps:

In the Initial steps we have gone through the tutorial for Navigation to understand how to do gmapping and use SLAM by utilizing the topics MOVE_BASE and amcl topic.After this tutorial we understood how to send goals to Move_base topic and keep getting localised position using amcl topic. Once these informations are provided to the move base function , the next step of moving the bot to the desired goal point is achieved.

     

###################################################################

General description of Topics and Function


Sr.No| List of Topics       | Subscribed/Published      | Message type        | Message imported from package    | 
:---: | :---         	    | :---                      | :---                | :--- 	                         | 
1    | /goals       	    | Subscribed directly       | PointArray          | goal-publisher                   |
2    | /amcl                | Subscribed                | PostCovariance      | geometry_msgs
			      from launch file                           
3    | /move_base/status    | Subscribed                | GoalStatusArray     | sensor-msgs
4    | '/move_base/goal'    | Published                 |  MoveBaseActionGoal | geometry-msgs                    |

1. `/goals` : topic is subscribed directly from launch file to get the goal points which was published from remote controller 

2. `/move_base/status` : Gives status of the bot .Status of 3 indicates the goal reached stage

3. `/move_base/goal` : 

4. `/amcl` : Model state function which is continously providing x,y and theta value of robot while moving

5. `/cmd_vel` :This topic is being used to run the robot initially with 360degree in order to make the robot moving vectors locally 

####################################################################

Algorithm description:


In this section we are explainning how the flow of algorithm works. First we are subscribing the goals from goals topic.From which we are sending these goals to sort function in which we are sorting the goals before giving input to Move_base action..In sorting we are making one array of goals which includes x,y and reward points and these are added together and then squared.After squarred it is used as key in making one dictionary called my_dict. From this dictionary we have done  sorting in ascending order. After that this sorted array we gave input to OrderDict which will make the index of the sorted array to be fixed and then saved to sorted_dict . After making indexes for the sorted array fixed we took every single index of the that array given to input of target array . In that array we are taking x and y cordinates as input to x and y of MoveBaseAction() function. This sorting will be done parallely however before starting of the robot it is rotated 360 degree to make robot paricle to be clouded in one direction that is in forward direction of robot. from this robot will be localised in a prope manner.After this localisation robot start moving to the goal points which we gave earlier after sorting.After reaching goal with tolerance status of the MoveBase function will be activated to 3 automatically. Once status is made to 3 we have increment the next goal point of the sorted array. This will follow untill all the goals are reached.



####################################################################

Implementation:

1.Subscribing and Publishing the topics:





2.Sorting of Goals before giving to Move_base:




3.Giving inputs to MoveBaseAction function:



####################################################################

Problems and solutions:

1. Giving an input to MoveBaseFunction:


2. Soting of goal points:



3. Setting up the environment in RViz for getting amcl topic to be published:



 
####################################################################

References:

1. Tutorial from the construct team on Gmapping, Move_base
2. http://wiki.ros.org/gmapping accessed on 22 June 2020
3. http://wiki.ros.org/navigation/Tutorials/RobotSetup accessed on 15 June 2020
4. http://library.isr.ist.utl.pt/docs/roswiki/gmapping.html accessed on 1 July 2020
5. https://docs.python.org/3/tutorial/classes.html  accessed on 12 July 2020
6. https://github.com/ros-perception/slam_gmapping/tree/melodic-devel/gmapping accessed on  3 July 2020

####################################################################
