# Udacity capstone project dockerfile
FROM ros:kinetic-robot
LABEL maintainer="olala7846@gmail.com"

RUN apt-get update
#RUN apt-get install wget

# Install Dataspeed DBW https://goo.gl/KFSYi1 from binary
# adding Dataspeed server to apt
RUN sh -c 'echo "deb [ arch=amd64 ] http://packages.dataspeedinc.com/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-dataspeed-public.list'
#RUN wget https://bitbucket.org/DataspeedInc/dbw_mkz_ros/raw/default/dbw_mkz/scripts/sdk_install.bash
#RUN bash sdk_install.bash
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys FF6D3CDA
RUN apt-get update

# setup rosdep
RUN sh -c 'echo "yaml http://packages.dataspeedinc.com/ros/ros-public-'$ROS_DISTRO'.yaml '$ROS_DISTRO'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'$ROS_DISTRO'.list'
RUN rosdep update
RUN apt-get install -y ros-$ROS_DISTRO-dbw-mkz
RUN apt-get upgrade -y
# end installing Dataspeed DBW

# install python packages
RUN apt-get install -y python-pip
COPY requirements.txt ./requirements.txt
RUN pip install -r requirements.txt
RUN pip install -U Pillow

# install required ros dependencies
RUN apt-get install -y ros-$ROS_DISTRO-cv-bridge
RUN apt-get install -y ros-$ROS_DISTRO-pcl-ros
RUN apt-get install -y ros-$ROS_DISTRO-image-proc

# socket io
RUN apt-get install -y netbase

# tmux
RUN apt-get install -y tmux
RUN apt-get install -y less

RUN python -m pip install --upgrade pip
RUN python -m pip install jupyter

RUN mkdir /capstone
VOLUME ["/capstone"]
VOLUME ["/root/.ros/log/"]
WORKDIR /capstone/ros
