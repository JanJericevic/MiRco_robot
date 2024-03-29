#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Header
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult
from mir_rest_api.api import MirRestApi
from pprint import pprint

class MiR100:
    """simple MiR100 robot class
    """

    def __init__(self, api: MirRestApi=None):
        
        #action server
        self.client = actionlib.SimpleActionClient('mir100/move_base', MoveBaseAction)
        # rospy.loginfo("Waiting for move_base action server...")
        # wait = self.client.wait_for_server(rospy.Duration(5.0))
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # rospy.loginfo("Connected to move base server")

        #move base result subscriber
        self.result_sub = rospy.Subscriber("/mir100/move_base/result", MoveBaseActionResult, self.handle_result)

        #move base goal retry counter and bool
        self.retry_counter = 1
        self.retry_target = True

        #MiR REST api
        self.api = api

        #Get mission guids for color missions
        light_missions = self.api.missions_groups_group_name_missions_get("FE_robolab_light")
        self.magenta_color_guid = next(item for item in light_missions[1] if item["name"] == "show_magenta_light")["guid"]
        self.cyan_color_guid = next(item for item in light_missions[1] if item["name"] == "show_cyan_light")["guid"]

    def handle_result(self,msg):
        self.result_status = msg.status.status
        rospy.loginfo("Move base result: " + str(msg.status.text))
    
    def loginfo_magenta(self,msg):
        """Wrapper around rospy.loginfo to print messages in magenta color.
        """

        magenta_start = '\033[95m'
        color_reset = '\033[0m'
        rospy.loginfo(magenta_start + str(msg) + color_reset)
    
    def loginfo_cyan(self,msg):
        """Wrapper around rospy.loginfo to print messages in magenta color.
        """

        magenta_start = '\033[36m'
        color_reset = '\033[0m'
        rospy.loginfo(magenta_start + str(msg) + color_reset)
    
    def show_light(self, state: str):
        """Workaround for showing color indicators on MiR100. Infinite loop mission defined on web interface that are then triggered over REST api

        :param state: robot operational state
        :type color: str
        """
        self.api.status_state_id_put(3)

        if state == "planner":
            delete = self.api.mission_queue_delete()[0]
            if delete != 204:
                rospy.logwarn("Mission queue unsuccessfully deleted. Light indication aborted")
                return
            self.loginfo_cyan("Setting light indicator to: planner mode")
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
            
    def get_position_guids(self):
        """Get positions for the active map that were defined in the MiR web interface.
        """

        status = self.api.status_get()
        map_id = status[1]["map_id"]
        positions = self.api.maps_map_id_positions_get(map_id)[1]
        position_guids = []
        for position in positions:
            guid = position["guid"]
            position_guids.append(guid)
        
        return(position_guids)

    def get_position_data(self, position_guids):
        """get pose data of positions defined by position guids
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
