FROM ros:noetic

LABEL   maintainer="Tammer Barkouki" \
        name="thbarkouki/explainablerobot"

ENV DEBAIN_FRONTEND noninteractive

RUN apt update -y && apt upgrade -y \
    && apt install git python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
    # && apt clean autoclean && apt autoremove -y && rm -rf /var/lib/apt/lists/* \
    # && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

WORKDIR /root/catkin_ws
COPY explainBT.rosinstall .
RUN wstool init src \
    && wstool merge -t src explainBT.rosinstall \
    &&  wstool update -t src

###################


RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd $HOME/catkin_ws; catkin_make; cd ..'
