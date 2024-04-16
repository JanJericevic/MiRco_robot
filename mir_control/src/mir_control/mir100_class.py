#! /usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Header
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult
from mir_rest_api.api import MirRestApi
from pprint import pprint
from typing import List, Union
from mir_control.goal_teacher import GoalTeacher
from mir_control.srv import *

import numpy as np
import yaml
import math

class MiR100:
    """Simple MiR100 robot class
    """

    def __init__(self, use_api=False, api_uname="", api_pass="", mir_ip="192.168.12.20"):
        self.loginfo_magenta("Initializing MiR100 robot python commander")

        # get robot namespaces
        if rospy.has_param("/robot_namespace"):
            self.namespace = rospy.get_param("/robot_namespace")
        else:
            self.namespace = ""
        if rospy.has_param("/robot_base_namespace"):
            self.base_namespace = rospy.get_param("/robot_base_namespace")
        else:
            self.base_namespace = ""

        # action server
        self.client = actionlib.SimpleActionClient(self.namespace + self.base_namespace + "/move_base", MoveBaseAction)
        self.loginfo_magenta("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        self.loginfo_magenta("Connected to move base server")
        
        # send robot to target goal service
        self.goal_server = rospy.Service("~send_to_goal", GoToGoal, self.send_to_goal)
        self.send_srv_name = rospy.get_name()+ "/send_to_goal"
        rospy.wait_for_service(self.send_srv_name)
        self.loginfo_magenta("Send to goal service: " + self.send_srv_name)

        # move base result subscriber
        self.result_sub = rospy.Subscriber(self.namespace + self.base_namespace + "/move_base/result", MoveBaseActionResult, self.handle_result)
        # move base status subscriber
        # self.status_sub = rospy.Subscriber(self.namespace + self.base_namespace + "/move_base/status", GoalStatusArray, self.handle_status)

        # goal retry counter and bool
        self.retry_counter = 1
        self.retry_target = True

        # MiR REST api
        if use_api:
            self.api = MirRestApi(api_uname,api_pass, mir_ip)
        
        # Get mission guid for docking mission
        helper_missions = self.api.missions_groups_group_name_missions_get("MiRco_helper_missions")
        self.dock_to_vl_guid = next(item for item in helper_missions[1] if item["name"] == "dock_to_vl_marker")["guid"]

        # Get mission guids for color missions
        self.magenta_color_guid = next(item for item in helper_missions[1] if item["name"] == "show_magenta_light")["guid"]
        self.cyan_color_guid = next(item for item in helper_missions[1] if item["name"] == "show_cyan_light")["guid"]
        self.green_color_guid = next(item for item in helper_missions[1] if item["name"] == "show_green_light")["guid"]
        self.red_color_guid = next(item for item in helper_missions[1] if item["name"] == "show_red_light")["guid"]

        # initialize goal teacher
        self.gt = GoalTeacher()
        # wait for goal teacher services
        rospy.wait_for_service(self.gt.save_srv_name)
        rospy.wait_for_service(self.gt.get_srv_name)

        # save map positions to file
        self.loginfo_magenta("Saving map positions to file")
        self.save_positions_2_file()

        # end of robot initialization
        self.loginfo_magenta("MiR100 robot python commander initialization complete.")
        self.log_status_state()

    def handle_result(self,msg:MoveBaseActionResult) -> None:
        """Move base result callback function

        :param msg: move base action result msg
        :type msg: MoveBaseActionResult
        """
        self.result_status = msg.status.status
        self.result_status_text = msg.status.text
        if self.result_status == 2:
            self.loginfo_magenta("Move base: received a cancel request")
            return
        self.loginfo_magenta("Move base: " + str(msg.status.text))
    
    # def handle_status(self,msg:GoalStatusArray) -> None:
    #     """Move base status callback function

    #     :param msg: move base action status msg
    #     :type msg: GoalStatusArray
    #     """
    #     self.status_status = msg.status_list[0].status
    #     self.loginfo_magenta("Move base status: " + str(msg.status_list[0].text))
    
    def loginfo_magenta(self,msg:str) -> None:
        """Helper function. Print loginfo message with light magenta text

        :param msg: message
        :type msg: str
        """
        rospy.loginfo('\033[95m' + "MiR100: " + msg + '\033[0m')
    
    def dock_to_vl_marker(self):
        """Workaround for docking the robot. 
        
        We define a simple mission with a single docking action in the MiR web interface. We 
        """

        # put robot in 'ready' state
        self.api.status_state_id_put(3)
        delete = self.api.mission_queue_delete()[0]
        if delete != 204:
            rospy.logwarn("Mission queue unsuccessfully deleted. Docking aborted")
            return
        self.loginfo_magenta("Docking to vl marker")
        self.api.mission_queue_post(self.dock_to_vl_guid)
    
    def show_light(self, state: str) -> None:
        """Workaround for showing color indicators on MiR100. Infinite loop mission defined on web interface that are then triggered over REST api

        :param state: robot operational state
        :type color: str
        """
        # put robot in 'ready' state
        self.api.status_state_id_put(3)

        if state == "planner":
            delete = self.api.mission_queue_delete()[0]
            if delete != 204:
                rospy.logwarn("Mission queue unsuccessfully deleted. Light indication aborted")
                return
            self.loginfo_magenta("Setting light indicator to: planner mode")
            self.api.mission_queue_post(self.cyan_color_guid)

        elif state == "blocked_path":
            delete = self.api.mission_queue_delete()[0]
            if delete != 204:
                rospy.logwarn("Mission queue unsuccessfully deleted. Light indication aborted")
                return
            self.loginfo_magenta("Setting light indicator to: blocked path")
            self.api.mission_queue_post(self.magenta_color_guid)
        
        elif state == "goal_reached":
            delete = self.api.mission_queue_delete()[0]
            if delete != 204:
                rospy.logwarn("Mission queue unsuccessfully deleted. Light indication aborted")
                return
            self.loginfo_magenta("Setting light indicator to: goal reached")
            self.api.mission_queue_post(self.green_color_guid)
        
        elif state == "error":
            delete = self.api.mission_queue_delete()[0]
            if delete != 204:
                rospy.logwarn("Mission queue unsuccessfully deleted. Light indication aborted")
                return
            self.loginfo_magenta("Setting light indicator to: error")
            self.api.mission_queue_post(self.red_color_guid)

        elif state == "off":
            delete = self.api.mission_queue_delete()[0]
            self.loginfo_magenta("Light indicator turned off")

        else:
            rospy.logwarn("Invalid light indicator state selected. Possible states: {planner, blocked_path, goal_reached, error, off}")
            return
        rospy.sleep(2)
            
    def log_status_state(self) -> None:
        """Log MiR state
        """

        status_state = self.api.status_state_get()
        if status_state[0] == 200:
            state_id = status_state[1]['state_id']
            state_text = status_state[1]['state_text']
            self.loginfo_magenta("Current MiR100 state: " + state_text)
        else:
            self.logwarn("Unable to get MiR state")
    
    def euler_2_quaternion(self, roll: float, pitch: float, yaw: float) -> list:
        """Euler angle to a quaternion.

        :param roll: roll angle
        :type roll: float
        :param pitch: pitch angle
        :type pitch: float
        :param yaw: yaw angle
        :type yaw: float
        :return: quaternion angle representation
        :rtype: list
        """

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        # return a list of floats, not numpy floats
        return [qx.item(), qy.item(), qz.item(), qw.item()]
    
    def get_position_guids(self) -> list:
        """Get positions for the active map that were defined in the MiR web interface.

        :return: list of position guids
        :rtype: list
        """

        status = self.api.status_get()
        map_id = status[1]["map_id"]
        positions = self.api.maps_map_id_positions_get(map_id)[1]
        position_guids = []
        for position in positions:
            guid = position["guid"]
            position_guids.append(guid)
        
        return(position_guids)

    def get_position_data(self, position_guids) -> list:
        """Get pose data of positions defined by position guids.

        Both positions and markers from the web interface are categorized as position within the REST api. 
        Markers also have entry positions that also show up when looking up map positions. These marker entry positions have the same name as its parent marker.

        :return: list of positions data
        :rtype: list
        """

        positions = []
        for guid in position_guids:
            data = self.api.positions_guid_get(guid)[1]
            position = {}

            # check if position is marker entry position
            if data["type_id"] == 12:
                position["name"] = data["name"] + "_entry"
            else:
                position["name"] = data["name"]
            position["x"] = data["pos_x"]
            position["y"] = data["pos_y"]
            position["orientation"] = data["orientation"]

            positions.append(position)

        return positions
    
    def save_positions_2_file(self):
        """Save position data to file
        """

        guids = self.get_position_guids()
        positions = self.get_position_data(guids)

        if positions == []:
            self.loginfo_magenta("No saved positions detected on map")
            return

        # uses same file as goal teacher
        # read from file
        try:
            with open(self.gt.filename) as goal_file:
                saved_goals = yaml.load(goal_file, Loader=yaml.SafeLoader)
                if saved_goals == None:
                    saved_goals = {}
        except IOError as err:
            print("Could not open %s to read target goals" % self.gt.filename)
            print(err)
        
        for p in positions:
            # check if position already exists
            goal = {}
            if p["name"] in saved_goals:
                goal = saved_goals[p["name"]]
                self.loginfo_magenta("Position with name: '" + p["name"] + "' already exists. Overwriting...")

            # position position
            position = {}
            position["x"] = p["x"]
            position["y"] = p["y"]
            position["z"] = 0
            goal['position'] = position
            # position orientation
            euler = p["orientation"]
            quaternions = self.euler_2_quaternion(0,0,math.radians(euler))
            orientation={}
            orientation["x"] = quaternions[0]
            orientation["y"] = quaternions[1]
            orientation["z"] = quaternions[2]
            orientation["w"] = quaternions[3]
            goal['orientation'] = orientation

            saved_goals[p["name"]] = goal

            self.loginfo_magenta("Position saved as: '" + ["name"] + "'")

        # save to file
        with open(self.gt.filename, 'w') as goal_file:
            yaml.dump(saved_goals, goal_file)
        
        self.loginfo_magenta("Positions saved to file")

    def go_2_goal(self,target_goal:Pose) -> bool:
        """go to goal

        :param target_goal: target goal
        :type target_goal: Pose
        :return: move base action result
        :rtype: bool
        """
        # goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = target_goal
        self.current_goal = goal

        # self.loginfo_magenta("Going to goal: \n" + str(goal))

        # light indication
        self.show_light("planner")
        
        # Sends the goal to the action server
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action
        result = self.client.wait_for_result()
        while self.result_status == 4 and self.retry_target:
            self.goal_blocked = True
            result = self.retry_goal()
        
        if self.result_status == 3:
            self.show_light("goal_reached")
        if self.result_status == 4:
            self.show_light("blocked_path")

        return self.result_status_text
    
    def retry_goal(self):
        """Retry navigating to goal
        """

        if self.retry_counter >= 5:
            rospy.logwarn("Goal is still blocked, canceling...")
            self.retry_target = False
            self.client.cancel_all_goals()
            return False

        rospy.logwarn("Goal is blocked, retrying in 3s. Retry attempt (%i/5)", self.retry_counter)
        rospy.sleep(3)
        self.retry_counter += 1
        self.client.send_goal(self.current_goal)
        return
    
    def send_to_goal(self, request:GoToGoal) -> GoToGoalResponse:
        """Send robot to target goal over ROS service

        :param request: service request
        :type request: GoToGoal
        :return: service response
        :rtype: GoToGoalResponse
        """
        # wait for get goal service
        get_goal_service_name = self.gt.get_srv_name
        rospy.wait_for_service(get_goal_service_name)

        # get target goal
        try:
            get_goal_service = rospy.ServiceProxy(get_goal_service_name, GetGoal)
            target_goal = get_goal_service(request.name)            

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # convert to Pose msg
        goal = Pose()
        goal.position.x = target_goal.goal.position.x
        goal.position.y = target_goal.goal.position.y
        goal.position.z = target_goal.goal.position.z

        goal.orientation.x = target_goal.goal.orientation.x
        goal.orientation.y = target_goal.goal.orientation.y
        goal.orientation.z = target_goal.goal.orientation.z
        goal.orientation.w = target_goal.goal.orientation.w

        self.retry_target = True
        self.goal_blocked = False
        self.retry_counter = 1

        # send robot to goal
        result = self.go_2_goal(goal)
        # return service response
        if self.result_status == 2:
            return GoToGoalResponse("Move base: received a cancel request")
        return GoToGoalResponse("Move base: " + str(result))

def main():
    rospy.init_node('mir_robot_node')
    try:
        mir_ip = "192.168.65.179"
        mir = MiR100(use_api=True, api_uname="UserName", api_pass="Password", mir_ip=mir_ip)

    except rospy.ServiceException as e:
        print("error: %s" %e)

if __name__ == '__main__':
    main()
