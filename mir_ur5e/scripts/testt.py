#! /usr/bin/env python

from __future__ import print_function
import rospy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult,MoveBaseActionResult
from geometry_msgs.msg import PoseStamped

import tf

def go2goal(rel_goal):
    #get robot pose
    pose_stamped = PoseStamped()
    pose_stamped = rospy.wait_for_message('mir_pose', PoseStamped)

    header = pose_stamped.header
    pose = pose_stamped.pose

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = header.frame_id
    goal.target_pose.header.stamp = rospy.Time.now()

    #relative move
    goal.target_pose.pose = pose
    goal.target_pose.pose.position.x = pose.position.x + rel_goal

    # goal.target_pose.pose.orientation.x = 0
    # goal.target_pose.pose.orientation.y = 0
    # goal.target_pose.pose.orientation.z = 0
    # goal.target_pose.pose.orientation.w = 1

    # print("Goal:")
    # print(goal)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

def control_arm(data,args):
    print("result message received")
    global goal_idx

    goals = args[0]
    goal_idx = args[1]
    print(goal_idx)

    if data.status.status == 3:
        if goal_idx == len(goals) - 1:
            print("END GOAL REACHED")
            return
        
        print("ARM MOVEMENT")
        rospy.sleep(3)
        print("MOVING TO NEXT GOAL")
        goal_idx += 1
        result = go2goal(goals[goal_idx])



if __name__ == '__main__':
    rospy.init_node('test_client')
    global goal_idx
    
    goals = [2,-2]
    goal_idx = 0
    result_sub = rospy.Subscriber("move_base/result",MoveBaseActionResult,control_arm,(goals,goal_idx))
    
    try:
        result = go2goal(goals[goal_idx])
    except (rospy.ROSInterruptException):
        print("program interrupted before completion")