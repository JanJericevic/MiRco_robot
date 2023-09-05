# MiR100 environment setup and workflow

### Tested on:
- MiR100 robot
- software version 2.13.3.2

### Table of contents
- [Dependancies](#dependencies)
- [Packages overview](#overview)
- [Installation](#installation)
- [Working with Docker](#working-with-docker)
- [Connect to the robot](#connect-to-the-robot)
- [Convenience scripts](/convenience_scripts/README.md)

### Dependencies
- **`ROS noetic:`** full desktop version is preferred. Can install other variant dependant on your use case.
- **`rosdep:`** command-line tool for installing system dependencies. Follow these install [instructions](http://wiki.ros.org/rosdep).
- **`(optional) Docker:`** allow system flexibility. Viable option for no OS wide ROS install, or for working with multiple ROS versions. See [Working with Docker](#working-with-docker) for details. 

### Overview
- **`mir_robot:`** [DFKI](https://www.dfki.de/web) [package](https://github.com/DFKI-NI/mir_robot#mir_robot) containing the ROS driver and config files for MiR100 robot.
- **`mir_joy_teleop:`** joystick teleoperation package.
- **`mir_rest_api:`** MiR100 REST API. Allows direct requests or requests using a ROS service.

## Installation
We are using the source install of the `mir_robot` package. For other options see the package [github page](https://github.com/DFKI-NI/mir_robot#mir_robot).

```bash
# create a catkin workspace
$ mkdir -p ~/MiR100/ws/src
$ cd ~/MiR100/ws/src

# clone mir_robot into the catkin workspace
$ git clone -b noetic https://github.com/DFKI-NI/mir_robot.git

# clone this repository into the catkin workspace
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

## Connect to the robot
***NOTE:*** everytime before using the robot synchronize system time as best you can. In the MiR interface go to: "System" -> "Settings" -> "Date & Time". For an advanced setup see [this solution](https://github.com/DFKI-NI/mir_robot#advanced).

### Connect to the web interface
- connect to the MiR_R**** hotspot
- open mir.com (default IP = 192.168.12.20)
- log in to the web interface
- control the robot

### Connect to the robot with ROS
There are two ways of using ROS to connect to the MiR100 robot:
- connect to its internal `roscore`
- connect using `mir_driver`. This method runs `roscore` on the host computer, connecting to the MiR100 internal `roscore` over `ROS Bridge`.

#### Internal `roscore`
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

#### Using `mir_driver`
- When connected to the MiR_R**** hotspot:

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
***NOTE:** when using `mir_driver` to connect to the robot we send `geometry_msgs/Twist` type messages instead of `geometry_msgs/TwistStamped` to the `/cmd_vel` topic. That is because the `mir_driver` package expects messages of type `geometry_msgs/Twist` on the `/cmd_vel` topic and converts them to `geometry_msgs/TwistStamped` messages before sending the commands to the robot.*

- When connected to the same [outside network](#connect-the-robot-to-a-wifi-network) as the robot:

```bash
# launch mir_driver and set the robot IP
$ roslaunch mir_driver mir.launch mir_hostname:=<robot-IP>

# test the connection the same way as the hotspot connection
```

### Connect the robot to a WIFI network
You can connect the robot to an outside network:

- turn on the robot and connect to its hotspot
- go to System -> Settings -> WiFi
- select "Add connection"
- select the network and fill in required information
- when you're finished select "Add connection"
- robot IP is displayed under the network connection name. You can use this IP to access the web interface or with [`mir_driver`](#using-mir_driver)

## Controlling the robot
***NOTE:*** everytime before using the robot synchronize system time as best you can. In the MiR interface go to: "System" -> "Settings" -> "Date & Time". For an advanced setup see [this solution](https://github.com/DFKI-NI/mir_robot#advanced).

### Using a joystick
To control the robot with a joystick use the `mir_joy_teleop` package.  
As with connecting to the robot, there is a slight difference between sending the joystick commands directly to the robots internal `roscore` or with the `mir_driver`.

#### Internal `roscore`

```bash
# launch a joy_node and a teleop node
# default input device is js1
$ roslaunch mir_joy_teleop joy_teleop.launch

# specify input device - e.g. js2
$ roslaunch mir_joy_teleop joy_teleop.launch device:=js2
```

#### Using `mir_driver`

```bash
# launch a joy_node and a teleop node
# we specify the use of an external roscore (default value is 'internal')
# default input device is js1
$ roslaunch mir_joy_teleop joy_teleop.launch roscore:=external

# specify input device - e.g. js2
$ roslaunch mir_joy_teleop joy_teleop.launch device:=js2 roscore:=external
```

## Working with Docker:
The `mir_robot` package and in turn `mir_driver` use the external computer `roscore` to control the MiR100 robot. With the OS wide ROS install you can only simultaneously use either the external computer `roscore` for controlling the robot or connect to the internal MiR100 computer `roscore` for monitoring. Docker containers allow you flexibility in your setup: 
- use your computer to control the robot and a Docker image to connect to the robot `roscore` for monitoring and vice versa
- use Docker containers for both controlling and monitoring the robot
- no need for OS wide ROS install & possibility of switching between ROS versions  

### Install Docker engine
Full install instructions are available at Dockers [official website](https://docs.docker.com/engine/install/ubuntu/).
Make sure old versions od Docker engine are [uninstalled](https://docs.docker.com/engine/install/ubuntu/#uninstall-docker-engine).

```bash
# update the apt package index and install packages to allow apt to use a repository over HTTPS:
$ sudo apt-get update
$ sudo apt-get install ca-certificates curl gnupg

# add Docker’s official GPG key
$ sudo install -m 0755 -d /etc/apt/keyrings
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
$ sudo chmod a+r /etc/apt/keyrings/docker.gpg

# set up the repository
$ echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Update the apt package index
$ sudo apt-get update

# install the latest version
$ sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# verify the installation
$ sudo docker run hello-world
```

#### Optional post installation steps
All optional installation steps are available at Dockers [official website](https://docs.docker.com/engine/install/linux-postinstall/).  
Useful post installation step is to manage Docker as a non sudo user. This allow the omission of `sudo` in front of docker commands. When allowing this make sure to be aware of how this [impacts security of your system](https://docs.docker.com/engine/security/#docker-daemon-attack-surface).

```bash
# create the docker group
# on some Linux distributions, the system automatically creates this group when installing Docker Engine using a package manager
$ sudo groupadd docker

# add your user to the docker group
$ sudo usermod -aG docker $USER

# log out and log back in so that your group membership is re-evaluated
# or un the following command to activate the changes to groups
$ newgrp docker

# verify that you can run docker commands without sudo
$ docker run hello-world
```

### Building the image
If all you want is to connect to the MiR100 `roscore` for monitoring all you need is a ROS Docker image.  
We will build a custom ROS Docker image complete with the same ROS packages so you have a choice of running the project locally or using Docker containers.

```bash
# download the desired packages to the src of your workspace
# move the Dockerfile in MiR100_robolab folder to the src folder
$ cd ~/MiR100/ws/src/MiR100_robolab
$ mv ~/MiR100/ws/src/MiR100_robolab/Dockerfile ~/MiR100/ws/src

# build the Docker image
$ cd ~/MiR100/ws/src
$ docker build -t <image-name> --build-arg MYUID=$(id -u) --build-arg MYGID=$(id -g) --build-arg MYUSER=$(id -nu) --build-arg MYGROUP=$(id -ng) .

# list your built Docker images
# verify that your <image-name> is among the listed images
$ docker images
```

***NOTE:** if you're on a machine with no OS wide ROS install and don't have a `catkin ws` the steps remain the same. Move the `Dockerfile` to the root directory of your packages, then build the Docker image in that root directory. The build commands remain the same.*

To avoid permissions issues with shared files between the host computer and the image container, we create a user with `sudo` permissions inside the image (this is especially relevant during [development](#volume-mounting)). User profile can be changed when building the image (the `build-arg` listed above) and inside the `Dockerfile`.  
The current profile settings are: 
   
> ***username***: same as the host username that built the image  
> ***password***: same as the username

The `Dockerfile` creates a `catkin workspace` at `/home/<your-user>/ws` inside the image. The workspace is also set as the work directory of the image so it will be the starting point of every new container.

Depending on your use case you will use the Docker image during development (you plan to regularly modify your codebase) or you will only use it for deployment. For deployment you only need to copy your files once, which is what we have done until now. For a development setup see the [volume mounting](#volume-mounting) section.

**TODO**
- dockerfile names
- dockerignore
- entrypoints
- convenience scripts
- useful tags

### Running the image
You can open an interactive bash shell with:
```bash
docker run -it <image-name> bash
```
This is ok for simple tasks however more advanced tasks require additional commands. Below are explanations for specific commands. However, since the commands can get complicated, we recommend the use of convenience scripts for repetitive cases. Create your own or use one of [ours](/convenience_scripts/).

#### Networking
There are many options for the network settings of a container that you can read about [here](https://docs.docker.com/engine/reference/run/#network-settings). Depending on your application you may want to use another option, in our case we choose to use the host's network inside the container. Among other things, this allows containers to talk to each other (e.g. one container is running the `roscore`, the other subscribes to a topic).

```bash
docker run -it --net=host <image-name> bash
```

#### GUI applications
ROS workflow is full of visual tools, which means that we need graphics capabilities from inside the container. [ROS wiki](http://wiki.ros.org/docker/Tutorials/GUI) mentions a few possible methods. Here we take the **simple but unsecure** method using X server. We expose our `xhost` so that the container can render to the correct display by reading and writing though the X11 unix socket.

```bash
docker run -it --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device /dev/dri \
    <image-name> \
    bash
```

Before you start the container, you have to change access permissions to the X server. The easiest is to grant access to everyone, or you can [specify](https://manpages.ubuntu.com/manpages/lunar/en/man1/xhost.1.html) a specific user.

```bash
# disables access control
$ xhost +

# grants access to specific user
$ xhost +local:<user-name>
# or
$ xhost +SI:local:<user-name>
```

Changes to the X server access only persist until the next logout/login, but it is best practice to enable back the access control once you're finished working with the container.

```bash
# enables access control
$ xhost -
```

#### NVIDIA
Users of NVIDIA GPUs can download the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker), which allows them to build GPU accelerated containers or in some cases solves display issues if the above mentioned setup is not working. The "Graphics inside Docker Containers" paragraph of this [ ROS&Docker guide](https://roboticseabass.com/2021/04/21/docker-and-ros/) describes working with such images. 

```bash
# example of a docker run command for NVIDIA GPU enabled container
$ docker run -it --net=host --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device /dev/dri \
    <image-name> \
    bash
```
#### Volume mounting
***NOTE:** mounted files are available only at runtime. Any files needed for building the image should be copied before then.*

For development we want persistent files that are shared between the host machine and the containers, files that can be changed both from the host side and from inside the container, in a setup which does not require us to rebuild the image every time we modify our code. We achieve this using [volumes](https://docs.docker.com/storage/volumes/). To achieve the necessary permissions we create a `sudo` user inside the image (see "[Building the image](#building-the-image)" section).


##### Example development setup for a host with the OS wide ROS install:
```bash
$ docker run -it \
    --volume="/home/<host-user>/ws":"/home/<container-user>/ws":rw \
    <image-name> \
    bash
```

We mount our host side workspace `/home/<host-user>/ws` to the container side workspace `/home/<container-user>/ws` in read-write mode. This means that the two workspaces are connected. Any change that we make in either of the two will affect both.  
It is best practice to build the workspace every time you start a new container and of course, every time you make a change to the code base.

##### Example development setup for a host without the OS wide ROS install:
```bash
$ docker run -it \
    --volume="/home/<host-user>/ws/src":"/home/<container-user>/ws/src":rw \
    <image-name> \
    bash
```
Here we mount the `src` folder of our host side workspace `/home/<host-user>/ws/src` to the `src` folder of our container side workspace `/home/<container-user>/ws/src` in read-write mode. This means that the two folders are connected. Any change that we make in either of the two will affect both.  
However, **since only the `src` folders of the workspaces are connected, the workspace still needs to be built inside the container with `catkin_make`**. It is best practice to build the workspace every time you start a new container and of course, every time you make a change to the code base. 

TODO: entrypoints, can you mount whole workspace?

#### Input devices
To use input devices like joysticks inside the Docker container we have to mount it when starting the container.

```bash
docker run -it \
    --device /dev/input \
    <image-name> \
    bash
```

The above example mounts the whole input directory but you can also specify specific devices. Here we mount joystick 0.

```bash
docker run -it \
    --device /dev/input/js0 \
    <image-name> \
    bash
```

**OPTIONAL:** Some input devices require you to change their permissions so that they become accessible to your application (e.g. ["Configuring the Joystick"](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick) section of this ROS tutorial). To avoid repetative changing of permissions every time you plug them in, you can create a `udev` rule.

```bash
# with the desired input device unplugged
# list the input devices on your host
$ ls /dev/input
by-id    event0  event10  event3  event5  event7  event9  mouse0  mouse2
by-path  event1  event11  event2   event4  event6  event8  mice    mouse1

# plug in the desired input device
# again list the input devices to see your device name
$ ls /dev/input
by-id    event1   event20  event5  event8  mice    mouse2
by-path  event10  event19  event3   event6  event9  mouse0
event0   event11  event2   event4   event7  js0     mouse1
```

Using the above method, we find that our device name is `js0`. Now we create a `udev` rule. Add a file `/etc/udev/rules.d/99-userdev-input.rules` with:

```bash
KERNEL=="js0", SUBSYSTEM=="input", ACTION=="add", RUN+="/usr/bin/setfacl -m o:rw $env{DEVNAME}"
```

This udev rule uses `ACL` to set the read-write permissions of the input device `js0` for the `others` group. You can modify the rule using `ACL` commands.