# ROS noetic image
FROM osrf/ros:noetic-desktop-full

# install network tools, sudo and pip
RUN apt update \
    && apt upgrade -y \
    && apt install -y net-tools wireless-tools iproute2 arp-scan nmap sudo python3-pip

# update python packages and install virtualenv
RUN python3 -m pip install --upgrade pip setuptools wheel \
    && python3 -m pip install virtualenv

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

# update and install packages
RUN apt update \
    && apt upgrade -y \
    && apt install -y --no-install-recommends build-essential python3-rosdep python3-catkin-lint python3-catkin-tools ros-noetic-joy \
    && python3 -m pip install requests \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# create a user (same as on host that builds the image)
# this is to avoid permission issues with shared files created inside the container
# user name is host user name. user password is same as user name
RUN groupadd $MYGROUP -g $MYGID \
    && useradd -m $MYUSER -u $MYUID -g $MYGID -d /home/${MYUSER} \
    && echo "$MYUSER:$MYUSER" | chpasswd && adduser $MYUSER sudo

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
    && DEBIAN_FRONTEND=noninteractive rosdep install --from-paths mir_robot/src -i -y --rosdistro noetic \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# change back to user
USER $MYUSER

# build all packages in the workspace 
RUN source "/opt/ros/noetic/setup.bash" \
    && cd /home/${MYUSER}/ws/src\
    && catkin_init_workspace \
    && cd /home/${MYUSER}/ws \
    && catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo

# add workspace to bashrc
RUN echo "source /home/${MYUSER}/ws/devel/setup.bash" >> ~/.bashrc

# source ROS and workspace
RUN source "/opt/ros/noetic/setup.bash" \
    && source "/home/${MYUSER}/ws/devel/setup.bash" 
