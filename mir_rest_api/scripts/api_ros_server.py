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