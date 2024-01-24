# ROS noetic image
FROM osrf/ros:noetic-desktop-full

# update and install packages
RUN apt update \
    && apt upgrade -y \
    && apt install -y net-tools wireless-tools iproute2 arp-scan nmap sudo python3-pip nano git \
    && apt install -y --no-install-recommends build-essential python3-rosdep python3-catkin-lint python3-catkin-tools python3-wstool 
RUN apt install -y ros-noetic-moveit
# RUN apt update \
#     && apt install -y ros-noetic-universal-robots
RUN apt update \
    && apt install -y ros-noetic-ur-robot-driver ros-noetic-ur-calibration ros-noetic-ur-calibration-dbgsym
RUN apt update \
    && apt-get install -y ros-noetic-soem

# update and install python packages
RUN python3 -m pip install --upgrade pip setuptools wheel \
    && python3 -m pip install virtualenv requests scipy

# clean up the apt cache
RUN rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# source ROS with .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# user args
ARG MYUSER
ARG MYUID
ARG MYGROUP
ARG MYGID

# change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# create a user (same as on host that builds the image)
# this is to avoid permission issues with shared files created inside the container
# user name is host user name. user password is same as user name
RUN groupadd $MYGROUP -g $MYGID \
    && useradd -m $MYUSER -u $MYUID -g $MYGID -d /home/${MYUSER} \
    && echo "$MYUSER:$MYUSER" | chpasswd && adduser $MYUSER sudo
# add user to dialout group
RUN usermod -a -G dialout $MYUSER

# add user directiory to path
ENV PATH="/home/${MYUSER}/.local/bin:$PATH"

# change to user to set permission when creating workspace
USER $MYUSER

# create ROS workspace and set owner
COPY --chown=$MYUSER . /home/${MYUSER}/ws/src

# create python environmetns directory
RUN cd /home/${MYUSER} \
    && mkdir -p environments/mir_env

# set up the work directory
WORKDIR /home/${MYUSER}/ws

# change to root for dependencies install
USER root

# use rosdep to install all dependencies
RUN rosdep update \
    && apt update \
    && cd /home/${MYUSER}/ws \
    && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths src -i -y --ignore-src --rosdistro noetic \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# change back to user
USER $MYUSER

# build all packages in the workspace 
RUN source "/opt/ros/noetic/setup.bash" \
    && cd /home/${MYUSER}/ws \
    && catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo

# add workspace to bashrc
RUN echo "source /home/${MYUSER}/ws/devel/setup.bash" >> ~/.bashrc

# source ROS and workspace
RUN source "/opt/ros/noetic/setup.bash" \
    && source "/home/${MYUSER}/ws/devel/setup.bash" 
