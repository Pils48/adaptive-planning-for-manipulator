ARG from

FROM ${from}

RUN apt-get update && apt-get install -y apt-utils \
                                         lsb-release \
                                         mesa-utils \
                                         gnupg2 \
                                         net-tools \
                                         build-essential \
                                         wget \
                                         unzip \
                                         curl \
                                         git \
                                         mc \
                                         vim


# Timezone Configuration

ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone


# Install ROS desktop and Gazebo

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && DEBIAN_FRONTEND=noninteractive \
                      apt-get install -y ros-kinetic-desktop-full \
                                         ros-kinetic-moveit \
                                         ros-kinetic-rviz-visual-tools \
                                         python-rosdep \
                                         python-wstool \
                                         python-catkin-tools &&\
    rosdep init && rosdep update && \
    echo "source /opt/ros/kinetic/setup.bash"  >> ~/.bashrc && \
    echo "source /workspace/devel/setup.bash"  >> ~/.bashrc

# Install extra libraries
RUN apt-get install -y libvisp-dev libvisp-doc 
RUN apt-get install -y python-pip

