#! /usr/bin/env python

import rospy
import rospkg
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
from mir_control.msg import *

import numpy as np
import yaml
import math
import threading

class MiR100:
    """MiR100 robot class
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

        # used for the wait_for_result function
        self.done_condition = threading.Condition()
        
        # ----- SERVICES -----
        # send robot to target goal ROS service
        self.goal_server = rospy.Service("~send_to_goal", GoToGoal, self.send_to_goal)
        self.send_srv_name = rospy.get_name()+ "/send_to_goal"
        rospy.wait_for_service(self.send_srv_name)
        self.loginfo_magenta("Send to goal service: " + self.send_srv_name)

        # dock robot to VL marker ROS service
        self.dock_server = rospy.Service("~dock_to_vl_marker", DockToMarker, self.dock_to_vl_marker)
        self.dock_srv_name = rospy.get_name()+ "/dock_to_vl_marker"
        rospy.wait_for_service(self.dock_srv_name)
        self.loginfo_magenta("Dock to VL marker service: " + self.dock_srv_name)

        # get saved marker ROS service
        self.saved_markers_server = rospy.Service("~get_markers", GetMarkers, self.get_markers)
        self.saved_marker_srv_name = rospy.get_name()+ "/get_markers"
        rospy.wait_for_service(self.saved_marker_srv_name)
        self.loginfo_magenta("Get markers service: " + self.saved_marker_srv_name)

        # change marker offsets ROS service
        self.offsets_server = rospy.Service("~change_marker_offsets", ChangeOffsets, self.change_marker_offsets)
        self.offsets_srv_name = rospy.get_name()+ "/change_marker_offsets"
        rospy.wait_for_service(self.offsets_srv_name)
        self.loginfo_magenta("Change marker offsets service: " + self.offsets_srv_name)

        # move base result subscriber
        self.result_status = None
        self.result_sub = rospy.Subscriber(self.namespace + self.base_namespace + "/move_base/result", MoveBaseActionResult, self.handle_result)
        # move base status subscriber
        # self.status_sub = rospy.Subscriber(self.namespace + self.base_namespace + "/move_base/status", GoalStatusArray, self.handle_status)

        # goal retry counter and bool
        self.retry_counter = 1
        self.retry_target = True

        # MiR REST api
        if use_api:
            self.api = MirRestApi(api_uname,api_pass, mir_ip)
        
        # Get mission guid for docking helper mission
        helper_missions = self.api.missions_groups_group_name_missions_get("MiRco_helper_missions")
        self.dock_to_vl_guid = next(item for item in helper_missions[1] if item["name"] == "dock_to_vl_marker")["guid"]
        # Get action guid of docking action inside docking helper mission
        self.dock_to_vl_action = self.api.missions_mission_id_actions_get(self.dock_to_vl_guid)[1]
        self.dock_to_vl_action_guid = self.dock_to_vl_action[0]["guid"]

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

        # get a filename to save robot positions and markers
        # robot positions file
        self.rp_filename = self.gt.filename
        # robot markers file
        rospack = rospkg.RosPack()
        package = "mir_control"
        self.rm_filename = rospack.get_path(package) + "/config/markers.yaml"

        # save map positions to file
        self.loginfo_magenta("Saving map positions to file")
        self.save_positions()

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
    #     self.status_status_text = msg.status_list[0].text
    #     self.loginfo_magenta("Move base status: " + str(msg.status_list[0].text))
    
    def loginfo_magenta(self,msg:str) -> None:
        """Helper function. Print loginfo message with light magenta text

        :param msg: message
        :type msg: str
        """
        rospy.loginfo('\033[95m' + "MiR100: " + msg + '\033[0m')
    
    def wait_for_result(self, timeout = rospy.Duration()) -> str:
        """Wait for result function.

        When docking the robot with the helper mission (see dock_to_vl_marker function), MiR does not publish a goal to the move base action server. 
        Because of this the the actionlib.SimpleActionClient.wait_for_result() does not work.

        :param timeout: timeout, defaults to rospy.Duration()
        :type timeout: Duration, optional
        :return: result
        :rtype: str
        """

        timeout_time = rospy.get_rostime() + timeout
        loop_period = rospy.Duration(0.1)
        with self.done_condition:
            while not rospy.is_shutdown():
                time_left = timeout_time - rospy.get_rostime()
                if timeout > rospy.Duration(0.0) and time_left <= rospy.Duration(0.0):
                    break

                if self.result_status == 3 and self.result_status_text == "Goal reached.":
                    break

                if time_left > loop_period or timeout == rospy.Duration():
                    time_left = loop_period

                self.done_condition.wait(time_left.to_sec())

        return self.result_status_text
    
    
    def show_light(self, state: str) -> None:
        """Workaround for showing color indicators on MiR100 with ROS. Infinite loop mission defined on web interface that are then triggered over REST api

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
    
    def get_positions(self) -> list:
        """Get positions details for the active map that were defined in the MiR web interface.

        :return: list of positions
        :rtype: list
        """

        status = self.api.status_get()
        map_id = status[1]["map_id"]
        map_positions = self.api.maps_map_id_positions_get(map_id)[1]
        positions = []
        for map_position in map_positions:
            positions.append(self.api.positions_guid_get(map_position["guid"])[1])

        return positions
    
    def save_markers_2_file(self, markers: list) -> None:
        """Save markers and their offsets to file
        """

        if markers == []:
            self.loginfo_magenta("No markers detected on map")
            return

        # read from file
        try:
            with open(self.rm_filename) as markers_file:
                saved_markers = yaml.load(markers_file, Loader=yaml.SafeLoader)
                if saved_markers == None:
                    saved_markers = {}
        except IOError as err:
            rospy.logwarn("Could not open %s to read markers" % self.gt.filename)
            print(err)
        
        # extract marker info
        for m in markers:
            marker = {}
            # check if marker already exists
            if m["name"] in saved_markers:
                marker = saved_markers[m["name"]]
                self.loginfo_magenta("Marker with name: '" + m["name"] + "' already exists. Overwriting...")

            marker["guid"] = m["guid"]
            marker["type_id"] = m["type_id"]

            # get marker offsets
            marker_offsets = self.api.positions_pos_id_docking_offsets_get(m["guid"])[1]
            marker_offsets_guid = marker_offsets[0]["guid"]
            marker_offsets = self.api.docking_offsets_guid_get(marker_offsets_guid)[1]
            marker["x_offset"] = marker_offsets["x_offset"]
            marker["y_offset"] = marker_offsets["y_offset"]
            marker["orientation_offset"] = marker_offsets["orientation_offset"]
            marker["offsets_guid"] = marker_offsets_guid

            saved_markers[m["name"]] = marker
            self.loginfo_magenta("Marker saved as: '" + m["name"] + "'")

        # save to file
        with open(self.rm_filename, 'w') as markers_file:
            yaml.dump(saved_markers, markers_file)
        self.loginfo_magenta("Markers saved to file")
    
    def save_robot_positions_2_file(self, positions) -> None:
        """Save robot positions to file
        """

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
            rospy.logwarn("Could not open %s to read target goals" % self.gt.filename)
            print(err)
        
        for p in positions:
            goal = {}
            # check if position already exists
            if p["name"] in saved_goals:
                goal = saved_goals[p["name"]]
                self.loginfo_magenta("Position with name: '" + p["name"] + "' already exists. Overwriting...")

            # position position
            position = {}
            position["x"] = p["pos_x"]
            position["y"] = p["pos_y"]
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
            self.loginfo_magenta("Position saved as: '" + p["name"] + "'")

        # save to file
        with open(self.gt.filename, 'w') as goal_file:
            yaml.dump(saved_goals, goal_file)
        self.loginfo_magenta("Positions saved to file")
    
    def save_positions(self) -> None:
        """Sort positions and save them
        """

        # get positions
        positions  = self.get_positions()

        robot_positions = []
        vl_markers = []
        for position in positions:
            if position["type_id"] == 0:
                robot_positions.append(position)
            elif position["type_id"] == 11:
                vl_markers.append(position)
            elif position["type_id"] == 12:
                self.loginfo_magenta("Found marker entry position. Not saving...")
                continue
            else:
                rospy.logwarn("Found unknown position type. Currently only saving positions of types {'Robot position','VL-marker'} ")
                continue

        self.save_markers_2_file(vl_markers)
        self.save_robot_positions_2_file(robot_positions)

    def get_markers(self, request: GetMarkersRequest) -> GetMarkersResponse:
        """Get markers

        :param request: ROS servive request
        :type request: GetMarkersRequest
        :return: ROS service response. Return a list of markers
        :rtype: GetMarkersResponse
        """
        # read from file
        try:
            with open(self.rm_filename) as markers_file:
                saved_markers = yaml.load(markers_file, Loader=yaml.SafeLoader)
                if saved_markers == None:
                    self.loginfo_magenta("Markers file empty")
                    return
        except IOError as err:
            self.loginfo_magenta("Could not open %s to read markers" % self.gt.filename)
            print(err)

        markers = []
        for marker in saved_markers:
            m = Marker()
            m.name = marker
            m.x_offset = saved_markers[marker]["x_offset"]
            m.y_offset = saved_markers[marker]["y_offset"]
            m.orientation_offset = saved_markers[marker]["orientation_offset"]
            markers.append(m)
        
        return [markers]
    
    def change_marker_offsets(self, request: ChangeOffsetsRequest) -> ChangeOffsetsResponse:
        """Change the offsets of a VL marker

        :param request: ROS service request. Specifies the marker name and its offsets
        :type request: ChangeOffsetsRequest
        :return: ROS service response
        :rtype: ChangeOffsetsResponse
        """

        # read from file
        try:
            with open(self.rm_filename) as markers_file:
                saved_markers = yaml.load(markers_file, Loader=yaml.SafeLoader)
                if saved_markers == None:
                    self.loginfo_magenta("Markers file empty")
                    return
        except IOError as err:
            self.loginfo_magenta("Could not open %s to read markers" % self.gt.filename)
            print(err)

        if request.name not in saved_markers:
            saved_marker_names = list(saved_markers)
            msg = ChangeOffsetsResponse()
            msg.result = "Selected marker name is not valid. Known markers: " + str(saved_marker_names)
            rospy.logwarn(msg.result)
            return msg

        # get the offsets guid of the selected marker
        offsets_guid = saved_markers[request.name]["offsets_guid"]
        # set offsets
        offsets = {
            "orientation_offset": request.orientation_offset,
            "x_offset": request.x_offset,
            "y_offset": request.y_offset
        }
        self.api.docking_offsets_guid_put(offsets_guid,offsets)

        self.loginfo_magenta("Changed offsets of marker: " + request.name)
        # refresh marker value
        self.loginfo_magenta("Refreshing position values")
        positions  = self.get_positions()
        for position in positions:
            if position["name"] == request.name and position["type_id"] == 11:
                self.save_markers_2_file([position])
        
        msg = ChangeOffsetsResponse()
        msg.result = "Changed offsets of marker: " + request.name
        return msg



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
        if self.result_status == 3:
            return GoToGoalResponse("Move base: " + str(result))

    
    def dock_to_vl_marker(self, request: DockToMarkerRequest) -> DockToMarkerResponse:
        """Workaround for docking the robot with ROS. 
        
        We define a simple mission with a single docking action in the MiR web interface. We use this mission to dock to markers, each time only changing the marker in the docking action of the mission.

        :param request: dock to marker service request, specifying the marker name
        :type marker_name: DockToMarkerRequest

        :param request: ROS service request. Specifies marker name
        :type request: DockToMarkerRequest
        :return: ROS service response
        :rtype: DockToMarkerResponse
        """

        self.result_status = 1

        # read from file
        try:
            with open(self.rm_filename) as markers_file:
                saved_markers = yaml.load(markers_file, Loader=yaml.SafeLoader)
                if saved_markers == None:
                    self.loginfo_magenta("Markers file empty")
                    return
        except IOError as err:
            self.loginfo_magenta("Could not open %s to read markers" % self.gt.filename)
            print(err)

        # get the guid of the selected marker
        if request.name not in saved_markers:
            saved_marker_names = list(saved_markers)
            msg = DockToMarkerResponse()
            msg.result = "Selected marker name is not valid. Known markers: " + str(saved_marker_names)
            rospy.logwarn(msg.result)
            return msg
        marker_guid = saved_markers[request.name]["guid"]

        # we only change the marker used in the docking action
        # set the json msg
        action_msg = {
            "priority": 1,
            "parameters": [
                {
                "value": str(marker_guid),
                "input_name": None,
                "guid": str(marker_guid),
                "id": "marker"
                }
            ]
        }

        # change docking action 
        change = self.api.missions_mission_id_actions_guid_put(self.dock_to_vl_guid, self.dock_to_vl_action_guid, action_msg)

        # put robot in 'ready' state
        self.api.status_state_id_put(3)
        # delete mission queue
        delete = self.api.mission_queue_delete()[0]
        if delete != 204:
            rospy.logwarn("Mission queue unsuccessfully deleted. Docking aborted")
            return
        # dock to selected marker using the helper mission
        self.loginfo_magenta("Docking to vl marker: " + request.name)
        self.api.mission_queue_post(self.dock_to_vl_guid)

        # Waits for the server to finish performing the action
        result = self.wait_for_result()
        # return service response
        if self.result_status == 2:
            return DockToMarkerResponse("Move base: " + str(result))
        if self.result_status == 3:
            return DockToMarkerResponse("Move base: " + str(result))
    

def main():
    rospy.init_node('mir_robot_node')
    try:
        mir_ip = "192.168.65.179"
        mir = MiR100(use_api=True, api_uname="UserName", api_pass="Password", mir_ip=mir_ip)

    except rospy.ServiceException as e:
        print("error: %s" %e)

if __name__ == '__main__':
    main()
