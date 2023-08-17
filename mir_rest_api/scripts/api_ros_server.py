#!/usr/bin/env python

"""REST API ROS service server script.

Server handles REST method requests as service calls and forwards responses.
Service calls and responses are of type 'Rest' service:

string method
string header
string endpoint
string json
---
int64 status_code
string response
"""

import rospy
import json
import requests
import pprint
from mir_rest_api.srv import *

def handle_request(req: RestRequest) -> RestResponse:
    """Handles ROS service call, makes REST request and returns ROS service response

    :param req: ROS service call 
    :type req: RestRequest
    :return: ROS service response
    :rtype: RestResponse
    """
    if req.method == "GET":
        try:
            rospy.loginfo("Sending REST GET request")
            response = requests.get(req.endpoint, headers=json.loads(req.header))
            rospy.loginfo("Sending REST GET response")
            return RestResponse(status_code = response.status_code, response = json.dumps(response.json()))
        except Exception as e:
            rospy.logerr(e)

    elif req.method == "DELETE":
        try:
            rospy.loginfo("Sending REST DELETE request")
            response = requests.delete(req.endpoint, headers=json.loads(req.header))
            rospy.loginfo("Sending REST DELETE response")
            return RestResponse(status_code = response.status_code, response = json.dumps({}))
        except Exception as e:
            rospy.logerr(e)

    elif req.method == "POST":
        try:
            rospy.loginfo("Sending REST POST request")
            response = requests.post(req.endpoint,json=json.loads(req.json), headers=json.loads(req.header))
            rospy.loginfo("Sending REST POST response")
            return RestResponse(status_code = response.status_code, response = json.dumps(response.json()))
        except Exception as e:
            rospy.logerr(e)
    
    elif req.method == "PUT":
        try:
            rospy.loginfo("Sending REST PUT request")
            response = requests.put(req.endpoint,json=json.loads(req.json), headers=json.loads(req.header))
            rospy.loginfo("Sending REST PUT response")
            return RestResponse(status_code = response.status_code, response = json.dumps(response.json()))
        except Exception as e:
            rospy.logerr(e)
    
    else:
        rospy.loginfo("Incorrect REST method!")

def main():
    rospy.init_node('mir_rest_api_server')
    s = rospy.Service('mir_rest_api_service', Rest, handle_request)
    rospy.loginfo("REST API service ready")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass