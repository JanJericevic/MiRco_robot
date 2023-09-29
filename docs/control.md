# Controlling the robot

***NOTE:*** everytime before using the robot synchronize system time as best you can. In the MiR interface go to: "System" -> "Settings" -> "Date & Time". For an advanced setup see [this solution](https://github.com/DFKI-NI/mir_robot#advanced).

## Using a joystick
To control the robot with a joystick use the `mir_joy_teleop` package.  
As with connecting to the robot, there is a slight difference between sending the joystick commands directly to the robots internal `roscore` or with the `mir_driver`.

### Internal `roscore`

```bash
# launch a joy_node and a teleop node
# default input device is js1
$ roslaunch mir_joy_teleop joy_teleop.launch

# specify input device - e.g. js2
$ roslaunch mir_joy_teleop joy_teleop.launch device:=js2
```

### Using `mir_driver`

```bash
# launch a joy_node and a teleop node
# we specify the use of an external roscore (default value is 'internal')
# default input device is js1
$ roslaunch mir_joy_teleop joy_teleop.launch roscore:=external

# specify input device - e.g. js2
$ roslaunch mir_joy_teleop joy_teleop.launch device:=js2 roscore:=external
```

TODO