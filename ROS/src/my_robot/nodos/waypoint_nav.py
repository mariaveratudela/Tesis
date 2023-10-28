#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

global pos
global orientacion
pos=[0,0,0]
orientacion0=0
orientacion1=0
orientacion2=0
orientacion3=0

def callback(msg):
	global pos
	global orientacion
	pos[0] = msg.pose.pose.position.x
	pos[1] = msg.pose.pose.position.y
	pos[2] = msg.pose.pose.position.z
	orientacion0 = msg.pose.pose.orientation.x
	orientacion1 = msg.pose.pose.orientation.y
	orientacion2 = msg.pose.pose.orientation.z
	orientacion3 = msg.pose.pose.orientation.w


class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos,quat):
        rospy.Subscriber("/odom", Odometry, callback)
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        
        navigator = GoToPose()

        # Customize the following values so they are appropriate for your location
        position = {'x': 11.9, 'y' :2.2}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0, 'r4' : 1.000}


        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position,quaternion)
        #success = navigator.goto(position)
        if success:
            rospy.loginfo("I'm here now... (%s, %s) pose", position['x'], position['y'])
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(3)

        position = {'x': 11.9, 'y' :2.925}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0, 'r4' : 1.000}


        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position,quaternion)
        #success = navigator.goto(position)
        if success:
            rospy.loginfo("I'm here now... (%s, %s) pose", position['x'], position['y'])
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(3)

        #position = {'x': 2.5, 'y' :-2.5}
        position = {'x': 11.9, 'y' :7.995}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0, 'r4' : 1.000}

  
        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position,quaternion)

        if success:
            rospy.loginfo("I'm here now... (%s, %s) pose", position['x'], position['y'])
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(3)

        position = {'x': 11.9, 'y' :9.945}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0, 'r4' : 1.000}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position,quaternion)

        if success:
            rospy.loginfo("I'm here now... (%s, %s) pose", position['x'], position['y'])
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(3)

        position = {'x': 7.605, 'y' :6.435}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0, 'r4' : 1.000}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position,quaternion)

        if success:
            rospy.loginfo("I'm here now... (%s, %s) pose", position['x'], position['y'])
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(3)

        position = {'x': 3.315, 'y' :6.045}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0, 'r4' : 1.000}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position,quaternion)

        if success:
            rospy.loginfo("I'm here now... (%s, %s) pose", position['x'], position['y'])
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(3)

        position = {'x': 2.145, 'y' :6.435}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0, 'r4' : 1.000}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position,quaternion)


        if success:
            rospy.loginfo("I'm here now... (%s, %s) pose", position['x'], position['y'])
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(3)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
