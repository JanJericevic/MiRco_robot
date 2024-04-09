#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Header
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult
from mir_rest_api.api import MirRestApi
from pprint import pprint
from typing import List, Union
from mir_control.goal_teacher import GoalTeacher

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

        # move base result subscriber
        self.result_sub = rospy.Subscriber(self.namespace + self.base_namespace + "/move_base/result", MoveBaseActionResult, self.handle_result)

        # move base goal retry counter and bool
        self.retry_counter = 1
        self.retry_target = True

        # MiR REST api
        if use_api:
            self.api = MirRestApi(api_uname,api_pass, mir_ip)

        # Get mission guids for color missions
        light_missions = self.api.missions_groups_group_name_missions_get("FE_robolab_light")
        self.magenta_color_guid = next(item for item in light_missions[1] if item["name"] == "show_magenta_light")["guid"]
        self.cyan_color_guid = next(item for item in light_missions[1] if item["name"] == "show_cyan_light")["guid"]

        # initialize goal teacher
        self.gt = GoalTeacher()
        # wait for goal teacher services
        rospy.wait_for_service(self.gt.save_srv_name)
        rospy.wait_for_service(self.gt.get_srv_name)

        # end of robot initialization
        self.log_status_state()
        self.loginfo_magenta("MiR100 robot python commander initialization complete.")

    def handle_result(self,msg:MoveBaseActionResult) -> None:
        """Move base result callback function

        :param msg: move base action result msg
        :type msg: MoveBaseActionResult
        """
        self.result_status = msg.status.status
        rospy.loginfo("Move base result: " + str(msg.status.text))
    
    def loginfo_magenta(self,msg:str) -> None:
        """Helper function. Print loginfo message with light magenta text

        :param msg: message
        :type msg: str
        """
        rospy.loginfo('\033[95m' + msg + '\033[0m')
    
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

        elif state == "off":
            delete = self.api.mission_queue_delete()[0]

        else:
            rospy.logwarn("Invalid light indicator state selected. Possible states: {planner, blocked_path, off}")
            return
            
    def log_status_state(self) -> None:
        """Log MiR state
        """

        status_state = self.api.status_state_get()
        if status_state[0] == 200:
            state_id = status_state[1]['state_id']
            state_text = status_state[1]['state_text']
            self.loginfo_magenta("MiR100 state: " + state_text)
        else:
            self.logwarn("Unable to get MiR state")
    
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
        """get pose data of positions defined by position guids

        :return: list of positions data
        :rtype: list
        """

        positions = []
        for guid in position_guids:
            data = self.api.positions_guid_get(guid)[1]
            position = {}

            position["name"] = data["name"]
            position["x"] = data["pos_x"]
            position["y"] = data["pos_y"]
            position["orientation"] = data["orientation"]

            positions.append(position)

        return positions

    def pose_msg_from_yaml(self, yaml):
        """Transfom yaml syntax to ROS Pose msg
        """

        pose_msg = Pose()
        pose_msg.position.x = yaml["position"]["x"]
        pose_msg.position.y = yaml["position"]["y"]
        pose_msg.position.z = yaml["position"]["z"]
        
        pose_msg.orientation.x = yaml["orientation"]["x"]
        pose_msg.orientation.y = yaml["orientation"]["y"]
        pose_msg.orientation.z = yaml["orientation"]["z"]
        pose_msg.orientation.w = yaml["orientation"]["w"]

        return pose_msg
    
    def get_goals_from_parameter(self, parameter:str):
        """Get target goals from parameter server
        """

        target_goals = PoseArray()
        if rospy.has_param(parameter):
            goals = rospy.get_param(parameter)
            for g in goals:
                target_goal = self.pose_msg_from_yaml(goals[g])
                target_goals.poses.append(target_goal)
            rospy.loginfo("Fetched target goals from parameter server.")

        else:
            rospy.logerr("Could not fetch target goals from parameter server!")
            rospy.signal_shutdown("Could not fetch target goals from parameter server!")
        
        target_goals.header.stamp = rospy.Time.now()
        return target_goals

    def go2goal(self,target_goal):
        """go to goal
        """

        # goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = target_goal

        self.loginfo_magenta(" >>> Going to goal: \n" + str(goal))

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        result = self.client.wait_for_result()
        return result
    
    def retry_goal(self):
        """Retry navigating to goal
        """

        if self.retry_counter >= 5:
            rospy.logwarn("Goal is still blocked, moving to next target.")
            self.retry_target = False
            self.client.cancel_goal()
            return

        rospy.logwarn("Goal is blocked, retrying in 3s. Retry attempt (%i/5)", self.retry_counter)
        rospy.sleep(3)
        self.retry_counter += 1
        result = self.go2goal(self.current_goal)
        return result

    def go2goals(self, target_goals):
        """go to series of goals
        """

        for tg in target_goals.poses:
            self.show_light("planner")
            self.retry_target = True
            self.retry_counter = 1
            self.current_goal = tg
            self.goal_blocked = False
            result = self.go2goal(self.current_goal)

            while self.result_status == 4 and self.retry_target:
                self.goal_blocked = True
                result = self.retry_goal()
                
            rospy.sleep(3)
        self.show_light("off")

def main():
    rospy.init_node('mir_robot_node')
    try:
        # mir_ip = "192.168.65.179"
        # api = MirRestApi("Distributor","distributor",mir_ip)
        api = MirRestApi("Distributor","distributor")
        mir = MiR100(api)

        # position_guids = mir.get_position_guids()
        # positions = mir.get_position_data(position_guids)
        # pprint(positions)

        # target_goals = mir.get_goals_from_parameter("mir100/pick_and_place/target_goals")
        # mir.go2goals(target_goals)

        # mir.show_light("planner")
        # rospy.sleep(3)
        # mir.show_light("blocked_path")
        # rospy.sleep(3)
        # mir.show_light("off")

    except rospy.ServiceException as e:
        print("error: %s" %e)

if __name__ == '__main__':
    main()
