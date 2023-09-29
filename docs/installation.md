# Installation

## Dependencies
- **`ROS noetic:`** full desktop version is preferred. Can install other variant dependant on your use case.
- **`rosdep:`** command-line tool for installing system dependencies. Follow these install [instructions](http://wiki.ros.org/rosdep).
- **`(optional) Docker:`** allows system flexibility. Viable option for no OS wide ROS install, or for working with multiple ROS versions. See [Working with Docker](#working-with-docker) for details.

## Packages overview
For this setup we will be installing 3 packages: 

- **`mir_robot:`** [DFKI](https://www.dfki.de/web) [package](https://github.com/DFKI-NI/mir_robot#mir_robot) containing the ROS driver and config files for MiR100 robot.
- **`mir_joy_teleop:`** joystick teleoperation package.
- **`mir_rest_api:`** MiR100 REST API. Allows direct requests or requests using a ROS service.

## Workspace setup
We are using the source install of the `mir_robot` package. For other options see the package [github page](https://github.com/DFKI-NI/mir_robot#mir_robot).

```bash
# create a catkin workspace
$ mkdir -p ~/MiR100/ws/src
$ cd ~/MiR100/ws/src

# clone mir_robot into the catkin workspace
$ git clone -b noetic https://github.com/DFKI-NI/mir_robot.git

# clone MiR100_robolab into the catkin workspace
$ git clone -b main https://github.com/JanJericevic/MiR100_robolab.git

# update and install packages
$ sudo apt update \
&& sudo apt upgrade -y \
&& sudo apt install -y --no-install-recommends python3-catkin-lint python3-catkin-tools

# use rosdep to install all dependencies
$ cd ~/MiR100/ws/ \
&& rosdep update \
&& rosdep install --from-paths src -i -y --rosdistro noetic 

# build all the packages in the catkin workspace
$ source /opt/ros/noetic/setup.bash \
&& cd ~/MiR100/ws/src \
&& catkin_init_workspace \
&& cd ~/MiR100/ws/ \
&& catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo

# source the workspace and add it to the .bashrc
$ source ~/MiR100/ws/devel/setup.bash \
&& echo "source ~/MiR100/ws/devel/setup.bash" >> ~/.bashrc
```

***NOTE:** if you wish to connect to the robot on a computer with no OS wide ROS install, see the ["Working with Docker" instructions.](TODO)*