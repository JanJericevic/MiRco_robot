# MiR100 environment setup and workflow

### Table of contents
- [Dependancies](#dependencies)
- [Packages overview](#overview)
- [Installation](#installation)
- [Working with Docker](#working-with-docker)
- [Frequent use cases](#frequent-use-cases)
- [Convenience scripts](/convenience_scripts/README.md)

### Dependencies
- **`ROS noetic:`** full desktop version is preferred. Can install other variant dependant on your use case.
- **`rosdep:`** command-line tool for installing system dependencies. Follow these install [instructions](http://wiki.ros.org/rosdep).
- **`(optional) Docker:`** when using the external roscore for controlling and MiR100 internal roscore for monitoring the robot. See [Working with Docker](#working-with-docker) for details. 

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

### Test the installation
```bash
# test mir_driver
$ roslaunch mir_driver mir.launch

# test REST API
# direct requests
```

## Working with Docker:
The `mir_robot` package and in turn `mir_driver` use the external computer roscore to control the MiR100 robot. With the OS wide ROS install you can only simultaneously use either the external computer roscore for controlling the robot or connect to the internal MiR100 computer roscore for monitoring. Docker containers allow you flexibility in your setup: 
- use your computer to control the robot and a Docker image to connect to the robot roscore for monitoring and vice versa
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
If all you want is to connect to the MiR100 roscore for monitoring all you need is a ROS Docker image.  
We will build a custom ROS Docker image complete with the same ROS packages so you have a choice of running the project locally or using Docker containers.

```bash
# clone this repository and any other ROS packages you need to your workspace

# go to the src directory of the workspace
$ cd ~/MiR100/ws/src

# build the Docker image
$ docker build -t <image-name> --build-arg MYUID=$(id -u) --build-arg MYGID=$(id -g) --build-arg MYUSER=$(id -nu) --build-arg MYGROUP=$(id -ng) .

# list your built Docker images
$ docker images

# verify that your <image-name> is among the listed images
```

***NOTE:** if you're on a machine with no OS wide ROS install and don't have a `catkin ws`, build the Docker image at the root of the directory where you copied the ROS packages to. The build commands remain the same.*

To avoid permissions issues with shared files between the host computer and the image container, we create a user with `sudo` permissions inside the image (this is especially relevant during [development](#volume-mounting)). User profile can be changed when building the image (the `build-arg` mentioned above) and inside the Dockerfile.  
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
There are many options for the network settings of a container that you can read about [here](https://docs.docker.com/engine/reference/run/#network-settings). Depending on your application you may want to use another option, in our case we choose to use the host's network inside the container. Among other things, this allows containers to talk to each other (e.g. one container is running the roscore, the other subscribes to a topic).

```bash
docker run -it --net=host <image-name> bash
```

#### GUI applications
ROS workflow is full of visual tools, which means that we need graphics capabilities from inside the container. [ROS wiki](http://wiki.ros.org/docker/Tutorials/GUI) mentions a few possible methods. Here we take the **simple but unsecure** method using X server. We expose our xhost so that the container can render to the correct display by reading and writing though the X11 unix socket.

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

## Frequent use cases: