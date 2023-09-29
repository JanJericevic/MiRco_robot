# Connect to the robot

There are two possible ways to connect to the robot:  

- using the MiR web interface
- using ROS

## Connect to the MiR web interface
- connect to the MiR_R**** hotspot
- open mir.com (default IP = 192.168.12.20)
- use your credentials to log in to the web interface

## Connect to the robot with ROS
There are two ways of using ROS to connect to the MiR100 robot:

- connect to its internal `roscore` running on the MiR100 computer
- connect using `mir_driver`. This method runs `roscore` on the host computer, connecting to the MiR100 internal `roscore` over `ROS Bridge`.

### Internal `roscore`
You have to be connected to the robot hotspot (MiR_R****).  
Set the address of the master to the address of MiR100 internal `roscore` and test the connection.

```bash
# this needs to be done in every terminal
export ROS_MASTER_URI=http://192.168.12.20:11311
export ROS_HOSTNAME=<your IP: 192.168.12.**>

# test the connection
# test subscriber
$ rostopic list
$ rostopic echo /cmd_vel

# test publisher
$ rostopic pub -r 50 /cmd_vel geometry_msgs/TwistStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.2"
```

### Using `mir_driver`
- If connected to the MiR_R**** hotspot:

```bash
# launch mir_driver
$ roslaunch mir_driver mir.launch

# test the connection
# test subscriber
$ rostopic echo /amcl_pose

# test publisher
$ rostopic pub -r 50 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"
```

- If connected to the same [outside network](#connect-the-robot-to-a-wifi-network) as the robot:

```bash
# launch mir_driver and set the robot IP
$ roslaunch mir_driver mir.launch mir_hostname:=<robot-IP>

# test the connection the same way as the hotspot connection
```

***NOTE:*** when using `mir_driver` to connect to the robot we send `geometry_msgs/Twist` type messages instead of `geometry_msgs/TwistStamped` to the `/cmd_vel` topic. That is because the `mir_driver` package expects messages of type `geometry_msgs/Twist` on the `/cmd_vel` topic and converts them to `geometry_msgs/TwistStamped` messages before sending the commands to the robot.

## Connect the robot to a WIFI network
You can connect the robot to an outside network:

- turn on the robot and connect to its hotspot
- go to System -> Settings -> WiFi
- select "Add connection"
- select the network and fill in required information
- when you're finished select "Add connection"
- robot IP is displayed under the network connection name. You can use this IP to access the web interface or when using [`mir_driver`](#TODO)
