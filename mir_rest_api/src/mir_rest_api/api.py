#!/usr/bin/env python

"""MIR100 REST API

This API does not contain every endpoint in the MiR100 REST API!
It contains only endpoints useful to the author.

Instructions:
Don't forget to set robot REST authorization header.
Don't forget to set the robot IP if using other network than MiR100 internal network.
Authorization header is generated as: BASE64( <username>:SHA-256( <password> ) )

Two ways of sending REST requests:
    handle_request(): send direct REST request to the robot API
    handle_request_ros(): send REST request over ROS service. Needs ROS service server node running
"""

import requests
import sys
import rospy
from mir_rest_api.srv import *
import json
from pprint import pprint
import base64
import hashlib

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
    
    def generate_auth_head(self, usrname: str, password: str):
        """generate authorization header

        :param usrname: MiR interface username
        :type usrname: str
        :param password: MiR interface password
        :type password: str
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

        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
                response = requests.post(self.endpoint,json=self.json, headers=self.header)
                response = [response.status_code, response.json()]
                return response
            except Exception as e:
                # rospy.logerr(e)
                print(e) 

        elif self.method == "PUT":
            try:
                response = requests.put(self.endpoint,json=self.json, headers=self.header)
                response = [response.status_code, response.json()]
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
    # ---------- robot state ----------
    def status_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the robot status

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        """Modify the current state of the robot. Choices are: {3, 4, 11}, State: {Ready, Pause, Manual control}

        :param state_id: desired robot state
        :type state_id: int
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: list containing REST response status code and body
        :rtype: [int, dict]
        """

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
        """Toggle the current state of the robot between 'Ready'/'Executing' and 'Pause'. Choices are: {3, 4}, State: {Ready, Pause}

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
    def mission_groups_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of mission groups

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: list containing REST response status code and body
        :rtype: [int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/mission_groups"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    def mission_groups_group_id_missions_get(self,group_id: str, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of missions that belong to the group with the specified group ID

        :param group_id: mission group ID
        :type group_id: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: list containing REST response status code and body
        :rtype: [int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/mission_groups/" + group_id + "/missions"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

    def mission_queue_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of missions in the queue. Finished, failed, pending and executing missions will be displayed here

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/mission_queue/" + str(mission_queue_id) + "/actions/" + str(id)
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

    def missions_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of missions

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/missions/" + mission_id + "/actions/" + guid
        self.json = ""
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
        """

        self.method = "DELETE"
        self.endpoint = self.url + "/missions/" + mission_id + "/actions/" + guid
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
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
        :return: list containing REST response status code and body
        :rtype: [int, dict]
        """

        self.method = "DELETE"
        self.endpoint = self.url + "/maps/" + guid
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()
    
    # ---------- positions ----------
    def maps_map_id_positions_get(self, map_id: str, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of positions that belong to the map with the specified map ID

        :param map_id: map ID
        :type map_id: str
        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: list containing REST response status code and body
        :rtype: [int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/maps/" + map_id + "/positions"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

    def position_transition_lists_get(self, ros: bool = 0) -> [int, dict]:
        """Retrieve the list of position transition lists

        :param ros: use ROS service, defaults to 0
        :type ros: bool, optional
        :return: list containing REST response status code and body
        :rtype: [int, dict]
        """

        self.method = "GET"
        self.endpoint = self.url + "/position_transition_lists"
        self.json = ""
        if ros == 1:
            return self.handle_request_ros()
        else:
            return self.handle_request()

def main():

    print("""
    Set your MiR interface User Name and Password.
    If using other network than MiR100 internal network set robot IP.
    Authorization header is generated as: BASE64( <username>:SHA-256( <password> ) )
    """)

    # set the MiR100 ip
    ip = "193.2.178.59"

    # if using ROS service for REST requests
    # rospy.wait_for_service('mir_rest_api_service')

    try:
        api = MirRestApi("UserName", "Password")  
        # api = MirRestApi("UserName", "Password", ip) # when setting robot IP
        
        print(api.header)

    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

if __name__ == '__main__':
    main()