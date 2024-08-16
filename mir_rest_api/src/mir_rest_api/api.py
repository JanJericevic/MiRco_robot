#!/usr/bin/env python

# Copyright 2024, Jan Jerićević

# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions
# are met:

#     * Redistributions of source code must retain the above copyright 
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above 
#     copyright notice, this list of conditions and the following 
#     disclaimer in the documentation and/or other materials provided 
#     with the distribution.

#     * Neither the name of the copyright holder nor the names of its 
#     contributors may be used to endorse or promote products derived 
#     from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE.

"""MIR100 REST API

This API does not contain every endpoint in the MiR100 REST API!
It contains only endpoints useful to the author at the time of development.

Instructions:
Don't forget to set login credentials for the robot REST authorization header.
Don't forget to set the robot IP if using other network than MiR100 internal hotspot.

Two ways of sending REST requests:
    handle_request(): send direct REST request to the robot API
    handle_request_ros(): send REST request over ROS service. Requires ROS service server node running
"""

import requests
import sys
import rospy
from mir_rest_api.srv import *
import json
from pprint import pprint
import base64
import hashlib
from typing import List,Dict

class MirRestApi:
    """MiR REST API class

    :param usrname: MiR interface username. Used to generate authorization header.
    :type usrname: str
    :param password: MiR interface password. Used to generate authorization header.
    :type password: str
    :param ip: robot IP, defaults to "192.168.12.20"
    :type ip: str, optional
    """

    def __init__(self, usrname: str, password: str, ip: str = "192.168.12.20") -> None:
        self.url = "http://" + ip + "/api/v2.0.0"
        self.header = {
            'Content-Type': 'application/json',
            'Authorization': self.generate_auth_head(usrname, password)
            }

        # when sending requests over ROS services we need a service handle
        self.service_handle = rospy.ServiceProxy('mir_rest_api_service', Rest)
    
    def generate_auth_head(self, usrname: str, password: str) -> str:
        """generate authorization header

        :param usrname: MiR interface username
        :type usrname: str
        :param password: MiR interface password
        :type password: str
        :return: authorization string
        :rtype: str
        """

        hashed_pass = hashlib.sha256(password.encode('utf-8')).hexdigest()
        string = usrname + ":" + hashed_pass
        string_bytes = string.encode("ascii")
        base64_bytes = base64.b64encode(string_bytes)
        base64_string = base64_bytes.decode("ascii")
        auth_header = "Basic " + base64_string
        return auth_header
    
    def handle_request(self) -> [int, dict]:
        """Handle REST request

        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        if self.method == "GET":
            try:
                response = requests.get(self.endpoint, headers=self.header)
                response = [response.status_code, response.json()]
                return response
            except Exception as e:
                # rospy.logerr(e)
                print(e) 

        elif self.method == "DELETE":
            try:
                response = requests.delete(self.endpoint, headers=self.header)
                response = [response.status_code, {}]
                return response
            except Exception as e:
                # rospy.logerr(e)
                print(e) 
        
        elif self.method == "POST":
            try:
                response = requests.post(self.endpoint, json=self.json, headers=self.header)
                response = [response.status_code, response.json()]
                # if response[0] != 200:
                #     pprint(response[1])
                return response
            except Exception as e:
                # rospy.logerr(e)
                print(e) 

        elif self.method == "PUT":
            try:
                response = requests.put(self.endpoint, json=self.json, headers=self.header)
                response = [response.status_code, response.json()]
                # if response[0] != 200:
                #     pprint(response[1])
                return response
            except Exception as e:
                # rospy.logerr(e)
                print(e)  

        else:
            # rospy.loginfo("Incorrect REST method!")
            print("Incorrect REST method!")

    def handle_request_ros(self) -> RestResponse:
        """Handle REST request over ROS service. Needs ROS service server node running

        :return: ROS service response containing REST response status code and body
        :rtype: RestResponse
        """

        try:
            response = self.service_handle(self.method, json.dumps(self.header), self.endpoint, json.dumps(self.json))
            response = [response.status_code, json.loads(response.response)]
            return response
        except rospy.ServiceException as e:
            rospy.logerr(e)


    # MiR REST API endpoints:
    # -------- robot settings ---------
    def settings_groups_get(self, ros: bool = 0) -> [int, List[dict]]:
        """Retrieve a list with the settings groups

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: [int, List[dict]]
        """

        self.method = "GET"
        self.endpoint = self.url + "/setting_groups"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def settings_groups_id_get(self, group_id: int, ros: bool = 0) -> [int, List[dict]]:
        """Retrieve the list of settings from the settings group with the specified settings group ID

        :param group_id: settings group ID
        :type group_id: int
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: [int, List[dict]]
        """

        self.method = "GET"
        self.endpoint = self.url + "/setting_groups/" + str(group_id) + "/settings"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

    def settings_groups_name_get(self, group_name: str, ros: bool = 0) -> [int, List[dict]]:
        """Retrieve the list of settings from the settings group with the specified settings group name

        :param group_name: settings group name
        :type group_name: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: [int, List[dict]]
        """

        groups = self.settings_groups_get(ros)
        setting_group = next(item for item in groups[1] if item["name"] == group_name)
        setting_group = self.settings_groups_id_get(setting_group["id"],ros)
        return setting_group
    
    def setting_name_get(self, group_name: str, setting_name: str, ros: bool = 0) -> [int,dict]:
        """Retrieve the details of the setting with the specified settings group name and setting name

        :param group_name: settings group name
        :type group_name: str
        :param setting_name: setting name
        :type setting_name: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: [int,dict]
        """

        settings_list = self.settings_groups_name_get(group_name,ros)
        setting = next(item for item in settings_list[1] if item["name"] == setting_name)
        return [settings_list[0], setting]

    def setting_name_put(self, group_name: str, setting_name: str, setting_value: str, ros: bool = 0) -> [int,dict]:

        setting_id = self.setting_name_get(group_name, setting_name, ros)[1]["id"]

        self.method = "PUT"
        self.endpoint = self.url + "/settings/" + str(setting_id)
        self.json = {"value": str(setting_value)}
        if ros == 1:
            response =  self.handle_request_ros()
            return [response[0], {"name": response[1]["name"],"value": response[1]["value"]}]
        else:
            response =  self.handle_request()
            return [response[0], {"name": response[1]["name"],"value": response[1]["value"]}]
        
    # ---------- robot state ----------
    def status_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the robot status

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/status"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def status_put(self, json: dict, ros: bool = 0) -> [int, dict]:
        """Modify the robot status

        :param json: request body
        :type json: dict
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "PUT"
        self.endpoint = self.url + "/status"
        self.json = json
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def status_mode_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the current mode of the robot

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/status"
        self.json = ""
        if ros == 1:
            response =  self.handle_request_ros()
            return [response[0], {"mode_id": response[1]["mode_id"],"mode_text": response[1]["mode_text"], "mode_key_state":response[1]["mode_key_state"]}]
        else:
            response =  self.handle_request()
            return [response[0], {"mode_id": response[1]["mode_id"],"mode_text": response[1]["mode_text"], "mode_key_state":response[1]["mode_key_state"]}]

    def status_state_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the current state of the robot

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/status"
        self.json = ""
        if ros == 1:
            response =  self.handle_request_ros()
            return [response[0], {"state_id": response[1]["state_id"],"state_text": response[1]["state_text"]}]
        else:
            response =  self.handle_request()
            return [response[0], {"state_id": response[1]["state_id"],"state_text": response[1]["state_text"]}]

    def status_state_id_put(self, state_id: int, ros: bool = 0) -> [int, dict]:
        """Modify the current state of the robot. 
        
        Possible robot state_id: {3, 4, 5, 11}, State: {Ready, Pause, Executing,Manual control}.
        Using this method the user can only put state_id: {3, 4}, State: {Ready, Pause}.

        :param state_id: desired robot state
        :type state_id: int
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        if state_id !=3 and state_id != 4:
            print("INVALID INPUT. Select state_id=3 for 'Ready' or state_id=4 for 'Pause'")
            return

        self.method = "PUT"
        self.endpoint = self.url + "/status"
        self.json = {"state_id" : state_id}
        if ros == 1:
            response =  self.handle_request_ros()
            return [response[0], {"state_id": response[1]["state_id"],"state_text": response[1]["state_text"]}]
        else:
            response =  self.handle_request()
            return [response[0], {"state_id": response[1]["state_id"],"state_text": response[1]["state_text"]}]

    
    def status_state_id_toggle_put(self, ros: bool = 0) -> [int, dict]:
        """Toggle the current state of the robot between 'Ready'/'Executing' and 'Pause'.

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        status = self.status_state_get(ros)[1]
        if status["state_id"] == 3:
            state_id = 4
            # rospy.loginfo("Setting the robot state to: 'Pause'")
            print("Robot is in state: 'Ready'. Setting the robot state to: 'Pause'")
        elif status["state_id"] == 4:
            state_id = 3
            # rospy.loginfo("Setting the robot state to: 'Ready'")
            print("Robot is in state: 'Pause'. Setting the robot state to: 'Ready'/'Executing'")
        elif status ["state_id"] == 5:
            state_id = 4
            # rospy.loginfo("Setting the robot state to: 'Ready'")
            print("Robot is in state: 'Executing'. Setting the robot state to: 'Pause'")
        elif status ["state_id"] == 10:
            # rospy.loginfo("Setting the robot state to: 'Ready'")
            print("Robot is in state: 'EmergencyStop'. Unable to toggle robot status")
            return
        else:
            # rospy.logerr("Unable to toggle robot status")
            print("Unable to toggle robot status")
            return

        self.method = "PUT"
        self.endpoint = self.url + "/status"
        self.json = {"state_id" : state_id}
        if ros == 1:
            response =  self.handle_request_ros()
            return [response[0], {"state_id": response[1]["state_id"],"state_text": response[1]["state_text"]}]
        else:
            response =  self.handle_request()
            return [response[0], {"state_id": response[1]["state_id"],"state_text": response[1]["state_text"]}]

    # ---------- missions ----------
    def mission_groups_get(self, ros: bool = 0) -> [int, List[dict]]:
        """Retrieve the list of mission groups

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, List[dict]]
        """

        self.method = "GET"
        self.endpoint = self.url + "/mission_groups"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def mission_groups_group_id_missions_get(self, group_id: str, ros: bool = 0) -> [int, List[dict]]:
        """Retrieve the list of missions that belong to the group with the specified group ID

        :param group_id: mission group ID
        :type group_id: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, List[dict]]
        """

        self.method = "GET"
        self.endpoint = self.url + "/mission_groups/" + group_id + "/missions"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

    def missions_groups_group_name_missions_get(self, group_name: str, ros: bool = 0) -> [int, List[dict]]:
        """Retrieve the list of missions that belong to the group with the specified group name

        :param group_name: missions group name
        :type group_name: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: [int, List[dict]]
        """
        
        mission_groups = self.mission_groups_get(ros)
        group = next(item for item in mission_groups[1] if item["name"] == str(group_name))
        group_guid = group["guid"]

        missions = self.mission_groups_group_id_missions_get(group_guid, ros)
        return missions

    def missions_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of missions

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/missions"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def missions_guid_get(self, guid: str, ros: bool = 0) -> [int, dict]:
        """Retrieve the details about the mission with the specified GUID

        :param guid: mission GUID
        :type guid: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/missions/" + guid
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def missions_guid_delete(self, guid: str, ros: bool = 0) -> [int, dict]:
        """Erase the mission with the specified GUID

        :param guid: mission GUID
        :type guid: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "DELETE"
        self.endpoint = self.url + "/missions/" + guid
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

    def missions_guid_definition_get(self, guid: str, ros: bool = 0) -> [int, dict]:
        """Retrieve the mission with the specified GUID as an action definition that can be inserted in another mission

        :param guid: mission GUID
        :type guid: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/missions/" + guid + "/definition"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def missions_mission_id_actions_get(self, mission_id: str, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of actions that belong to the mission with the specified mission ID

        :param mission_id: mission ID
        :type mission_id: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/missions/" + mission_id + "/actions"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def missions_mission_id_actions_guid_get(self, mission_id: str, guid: str, ros: bool = 0) -> [int, dict]:
        """Retrieve the details about the action with the specified GUID that belongs to the mission with the specified mission ID

        :param mission_id: mission ID
        :type mission_id: str
        :param guid: action GUID
        :type guid: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/missions/" + mission_id + "/actions/" + guid
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def missions_mission_id_actions_guid_put(self, mission_id: str, guid: str, action_msg, ros: bool = 0) -> [int, dict]:
        """Modify the values of the action with the specified GUID that belongs to the mission with the specified mission ID

        :param mission_id: mission ID
        :type mission_id: str
        :param guid: action GUID
        :type guid: str
        :param action_msg: MiR action body
        :type action_msg: json
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "PUT"
        self.endpoint = self.url + "/missions/" + mission_id + "/actions/" + guid
        self.json = action_msg
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def missions_mission_id_actions_guid_delete(self, mission_id: str, guid: str, ros: bool = 0) -> [int, dict]:
        """Erase the action with the specified GUID from the mission with the specified mission ID

        :param mission_id: mission ID
        :type mission_id: str
        :param guid: action GUID
        :type guid: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "DELETE"
        self.endpoint = self.url + "/missions/" + mission_id + "/actions/" + guid
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    # ---------- mission queue ----------
    def mission_queue_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of missions in the queue. Finished, failed, pending and executing missions will be displayed here

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/mission_queue"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

    def mission_queue_post(self, mission_id: str, ros: bool = 0) -> [int, dict]:
        """Add a new mission to the mission queue. The mission will always go to the end of the queue

        :param mission_id: mission ID
        :type mission_id: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "POST"
        self.endpoint = self.url + "/mission_queue"
        self.json = {"mission_id": mission_id}
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def mission_queue_delete(self, ros: bool = 0) -> [int, dict]:
        """Abort all the pending and executing missions from the mission queue

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "DELETE"
        self.endpoint = self.url + "/mission_queue"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def mission_queue_id_get(self, id: int, ros: bool = 0) -> [int, dict]:
        """Retrieve the details about the mission with the specified ID in the mission queue

        :param id: mission ID in the mission queue
        :type id: int
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/mission_queue/" + str(id)
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def mission_queue_id_delete(self, id: int, ros: bool = 0) -> [int, dict]:
        """Abort the mission with the specified ID in the mission queue

        :param id: mission ID in the mission queue
        :type id: int
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "DELETE"
        self.endpoint = self.url + "/mission_queue/" + str(id)
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def mission_queue_mission_queue_id_actions_get(self, mission_queue_id: int, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of actions from the mission with the specified ID in the mission queue

        :param mission_queue_id: mission ID in the mission queue
        :type mission_queue_id: int
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/mission_queue/" + str(mission_queue_id) + "/actions"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def mission_queue_mission_queue_id_actions_id_get(self, mission_queue_id: int, id: int, ros: bool = 0) -> [int, dict]:
        """Retrieve the details about the action with the specified ID from the mission with the specified ID in the mission queue

        :param mission_queue_id: mission ID in the mission queue
        :type mission_queue_id: int
        :param id: action ID
        :type id: int
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/mission_queue/" + str(mission_queue_id) + "/actions/" + str(id)
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

    # ---------- maps----------
    def maps_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of maps

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/maps"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def sessions_session_id_maps_get(self, session_id: str, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of maps that belong to the session with the specified session ID. session_id = site_id

        :param session_id: site ID
        :type session_id: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/sessions/" + session_id + "/maps"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def maps_guid_get(self, guid: str, ros: bool = 0) -> [int, dict]:
        """Retrieve the details about the map with the specified GUID

        :param guid: map GUID
        :type guid: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/maps/" + guid
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def maps_guid_delete(self, guid: str, ros: bool = 0) -> [int, dict]:
        """Erase the map with the specified GUID

        :param guid: map GUID
        :type guid: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "DELETE"
        self.endpoint = self.url + "/maps/" + guid
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    # ---------- positions ----------
    def positions_post(self, position, ros: bool = 0) -> [int, dict]: 
        """Add a new position.

        Position json needs to have at least: {name, pos_x, pos_y, orientation, type_id, map_id}

        :param position: position json
        :type position: json
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: [int, dict]
        """

        self.method = "POST"
        self.endpoint = self.url + "/positions"
        self.json = position
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

    def maps_map_id_positions_get(self, map_id: str, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of positions that belong to the map with the specified map ID

        :param map_id: map ID
        :type map_id: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/maps/" + map_id + "/positions"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def positions_guid_get(self, guid: str, ros: bool = 0) -> [int,dict]:
        """Retrieve the details about the position with the specified GUID

        :param guid: The global id unique across robots that identifies this position
        :type guid: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: [int,dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/positions/" + guid
        self.json = ""

        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

    def positions_pos_id_docking_offsets_get(self, pos_id: str, ros: bool = 0) -> [int,dict]:
        """Retrieve the details of the docking offset of the position with the specified position ID

        :param pos_id: position ID
        :type pos_id: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: [int,dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/positions/" + pos_id + "/docking_offsets"
        self.json = ""

        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

    def position_transition_lists_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of position transition lists

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/position_transition_lists"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def position_types_id_get(self, id: str, ros: bool = 0) -> [int,dict]:
        """Retrieve the details about the position type with the specified type ID

        :param guid: position type ID
        :type guid: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: [int,dict]
        """

        """ Position Types:
        {'id': 0, 'name': 'Robot position', 'url': '/v2.0.0/position_types/0'},
        {'id': 1, 'name': 'Cart position', 'url': '/v2.0.0/position_types/1'},
        {'id': 2, 'name': 'Cart pickup position', 'url': '/v2.0.0/position_types/2'},
        {'id': 3, 'name': 'Cart left entry', 'url': '/v2.0.0/position_types/3'},
        {'id': 4, 'name': 'Cart right entry', 'url': '/v2.0.0/position_types/4'},
        {'id': 5, 'name': 'Shelf position', 'url': '/v2.0.0/position_types/5'},
        {'id': 6, 'name': 'Shelf short entry position', 'url': '/v2.0.0/position_types/6'},
        {'id': 7, 'name': 'MiRCharge 100/200', 'url': '/v2.0.0/position_types/7'},
        {'id': 8, 'name': 'MiRCharge 100/200 entry', 'url': '/v2.0.0/position_types/8'},
        {'id': 9, 'name': 'V-marker', 'url': '/v2.0.0/position_types/9'},
        {'id': 10, 'name': 'V-marker entry', 'url': '/v2.0.0/position_types/10'},
        {'id': 11, 'name': 'VL-marker', 'url': '/v2.0.0/position_types/11'},
        {'id': 12, 'name': 'VL-marker entry', 'url': '/v2.0.0/position_types/12'},
        {'id': 13, 'name': 'L-marker', 'url': '/v2.0.0/position_types/13'},
        {'id': 14, 'name': 'L-marker entry', 'url': '/v2.0.0/position_types/14'},
        {'id': 15, 'name': 'Emergency position', 'url': '/v2.0.0/position_types/15'},
        {'id': 16, 'name': 'Precision marker', 'url': '/v2.0.0/position_types/16'},
        {'id': 17, 'name': 'Precision marker entry', 'url': '/v2.0.0/position_types/17'},
        {'id': 18, 'name': 'Pallet rack', 'url': '/v2.0.0/position_types/18'},
        {'id': 19, 'name': 'Pallet rack entry position', 'url': '/v2.0.0/position_types/19'},
        {'id': 20, 'name': 'MiRCharge 500/1000', 'url': '/v2.0.0/position_types/20'},
        {'id': 21, 'name': 'MiRCharge 500/1000 entry', 'url': '/v2.0.0/position_types/21'},
        {'id': 22, 'name': 'Bar-marker', 'url': '/v2.0.0/position_types/22'},
        {'id': 23, 'name': 'Bar-marker entry', 'url': '/v2.0.0/position_types/23'},
        {'id': 25, 'name': 'elevator position', 'url': '/v2.0.0/position_types/25'},
        {'id': 26, 'name': 'elevator entry position', 'url': '/v2.0.0/position_types/26'},
        {'id': 42, 'name': 'Staging position', 'url': '/v2.0.0/position_types/42'},
        {'id': 106, 'name': 'Shelf long entry position', 'url': '/v2.0.0/position_types/106'}]]
        """

        self.method = "GET"
        self.endpoint = self.url + "/position_types/" + id
        self.json = ""

        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
        
    def position_types_get(self, ros: bool = 0) -> [int,dict]:
        """Retrieve a list of possible position types

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: [int,dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/position_types"
        self.json = ""

        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

    # ---------- actions ----------
    def actions_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of action definitions

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/actions"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def actions_action_type_get(self, action_type: str, ros: bool = 0) -> [int, dict]:
        """Retrieve the details about the action type. It displays the parameters of the action and the limits for the values among others

        :param action_type: action type
        :type action_type: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/actions/" + action_type 
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    # ---------- docking offsets ----------
    def docking_offsets_guid_get(self, guid: str, ros: bool = 0) -> [int, dict]:
        """Retrieve the details of the docking offset with the specified GUID

        :param guid: action docking offset guid
        :type guid: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/docking_offsets/" + guid 
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def docking_offsets_guid_put(self, guid: str, offsets, ros: bool = 0) -> [int, dict]:
        """Modify the values of the docking offset with the specified GUID

        :param guid: action docking offset guid
        :type guid: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: a list containing REST response status code and body
        :rtype: list[int, dict]
        """

        self.method = "PUT"
        self.endpoint = self.url + "/docking_offsets/" + guid 
        self.json = offsets
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

def main():

    print("""
    Set your MiR interface User Name and Password.
    If using other network than MiR100 internal network set robot IP.
    """)

    # set the MiR100 ip
    # ip = "192.168.65.179"

    # if using ROS service for REST requests
    # rospy.wait_for_service('mir_rest_api_service')

    try:
        # api = MirRestApi("UserName", "Password")  
        api = MirRestApi("Distributor", "distributor", ip) # when setting robot IP

        # example of REST request over ROS service
        # print(api.status_state_get(1))

    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

if __name__ == '__main__':
    main()