FROM ros:indigo-ros-base
# osrf/ros:indigo-desktop-full
LABEL maintainer "vvjagtap@wpi.edu"

SHELL ["/bin/bash", "-c"]
RUN sudo rm /bin/sh && sudo ln -s /bin/bash /bin/sh

# Create a user
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/whrl && \
    echo "whrl:x:${uid}:${gid}:Whrl,,,:/home/whrl:/bin/bash" >> /etc/passwd && \
    echo "whrl:x:${uid}:" >> /etc/group && \
    echo "whrl ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/whrl && \
    chmod 0440 /etc/sudoers.d/whrl && \
    chown ${uid}:${gid} -R /home/whrl

USER whrl
ENV HOME /home/whrl

# Installing general required packages
RUN sudo apt-get -y update && sudo apt-get install -y wget
# && sudo rm -rf /var/lib/apt/lists/

RUN rosdep update 

# Install srcsim and Gazebo7
RUN /bin/bash -c "echo 'source /opt/ros/indigo/setup.bash' >> ~/.bashrc"
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget -O - http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
RUN sudo sh -c 'echo "deb http://srcsim.gazebosim.org/src trusty main" > /etc/apt/sources.list.d/src-latest.list'
RUN wget -O - http://srcsim.gazebosim.org/src/src.key | sudo apt-key add -
RUN wget -O - https://bintray.com/user/downloadSubjectPublicKey?username=bintray | sudo apt-key add -
RUN sudo apt-get -y update && sudo apt-get install -y srcsim

##Environment Variables
#RUN /bin/bash -c "echo 'ROS_MASTER_URI=http://172.17.0.1:11311' >> ~/.bashrc"
#RUN /bin/bash -c "echo 'ROS_IP=`hostname -i`' >> ~/.bashrc"
#RUN /bin/bash -c "echo 'ROS_HOSTNAME=`hostname -i`' >> ~/.bashrc"

# Install Dependencies 

# Leaving this out of the installs since no vnc needed
# x11vnc xvfb icewm xz-utils cmake screen konsole\

RUN  sudo apt-get -y update && sudo apt-get install -y git \
  g++ vim nano wget  ca-certificates  ssh ruby ros-indigo-pcl-ros \
  x11vnc xvfb icewm lxpanel iperf xz-utils cmake screen terminator konsole\ 
  ros-indigo-pcl-conversions ros-indigo-moveit \
  ros-indigo-trac-ik ros-indigo-footstep-planner \
  ros-indigo-humanoid-localization ros-indigo-multisense-ros \
  ros-indigo-laser-assembler ros-indigo-robot-self-filter \
  ros-indigo-tf2-geometry-msgs ros-indigo-joint-state-publisher \
  ros-indigo-octomap-server ros-indigo-octomap \
  ros-indigo-joint-trajectory-controller \
  ros-indigo-image-transport \
  ros-indigo-joint-state-controller ros-indigo-position-controllers \
  ros-indigo-multimaster-fkie srcsim\
  ros-indigo-moveit-full ros-indigo-sbpl \
  ros-indigo-humanoid-nav-msgs ros-indigo-map-server ros-indigo-trac-ik* \
  ros-indigo-multisense-ros ros-indigo-robot-self-filter ros-indigo-octomap \
  ros-indigo-octomap-msgs ros-indigo-octomap-ros ros-indigo-gridmap-2d \
  software-properties-common python-software-properties debconf-i18n 
#  && sudo rm -rf /var/lib/apt/lists/

# Create a catkin workspace
RUN /bin/bash -c "source /opt/nasa/indigo/setup.bash && \
                  mkdir -p ~/indigo_ws/src && \
                  cd ~/indigo_ws/src && \
                  catkin_init_workspace && \
                  cd ~/indigo_ws/ && \
                  catkin_make && \
                  echo 'source ~/indigo_ws/devel/setup.bash' >> ~/.bashrc"

#Install jdk8 with javafx support
RUN sudo add-apt-repository -y ppa:webupd8team/java
RUN sudo apt-get update
RUN echo "oracle-java8-installer shared/accepted-oracle-license-v1-1 select true" | sudo debconf-set-selections
RUN sudo apt-get install -y oracle-java8-installer && sudo rm -rf /var/lib/apt/lists/

#set default java to version 8
RUN echo 'export JAVA_HOME=/usr/lib/jvm/java-8-oracle/' >> ~/.bashrc
RUN sudo rm /usr/lib/jvm/default-java
RUN sudo ln -s /usr/lib/jvm//usr/lib/jvm/java-8-oracle /usr/lib/jvm/default-java

RUN echo 'export IS_GAZEBO=true' >> ~/.bashrc
RUN sudo chown -R whrl:whrl /opt/ros/indigo/share/ihmc_ros_java_adapter
RUN mkdir -p ${HOME}/.ihmc; curl https://raw.githubusercontent.com/ihmcrobotics/ihmc_ros_core/0.8.0/ihmc_ros_common/configurations/IHMCNetworkParametersTemplate.ini > ${HOME}/.ihmc/IHMCNetworkParameters.ini

RUN sudo bash -c 'echo "@ros - rtprio 99" > /etc/security/limits.d/ros-rtprio.conf'
RUN sudo groupadd ros
RUN sudo usermod -a -G ros whrl

RUN wget -P /tmp/ http://gazebosim.org/distributions/srcsim/valkyrie_controller.tar.gz
RUN tar -xvf /tmp/valkyrie_controller.tar.gz -C $HOME
RUN rm /tmp/valkyrie_controller.tar.gz

RUN /bin/bash -c "echo 'export ROS_MASTER_URI=http://192.168.2.10:11311' >> ~/.bashrc"
RUN /bin/bash -c "echo 'export ROS_IP=192.168.2.10' >> ~/.bashrc"                  
RUN /bin/bash -c "source ~/.bashrc"

RUN wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
RUN mkdir -p $HOME/.gazebo/models
RUN tar -xvf /tmp/default.tar.gz -C $HOME/.gazebo/models --strip 1
RUN rm /tmp/default.tar.gz


# Clone ihmc repos locally
RUN mkdir ~/indigo_ws/src/ihmc_repos
RUN git clone https://github.com/WPI-Humanoid-Robotics-Lab/ihmc_ros_core.git ~/indigo_ws/src/ihmc_repos/ihmc_ros_core
RUN cd ~/indigo_ws/src/ihmc_repos/ihmc_ros_core && git checkout val-develop-messages-update
RUN git clone https://github.com/ihmcrobotics/ihmc_valkyrie_ros.git ~/indigo_ws/src/ihmc_repos/ihmc_valkyrie_ros
RUN cd ~/indigo_ws/src/ihmc_repos/ihmc_valkyrie_ros && git checkout release/0.10.0
RUN git clone https://github.com/ihmcrobotics/ihmc-ros-control.git ~/indigo_ws/src/ihmc_repos/ihmc_ros_control
RUN cd ~/indigo_ws/src/ihmc_repos/ihmc_ros_control && git checkout 0.5.0
# Compile the code
RUN /bin/bash -c "source ~/.bashrc && cd ~/indigo_ws && sudo rm -rf build devel && catkin_make"
RUN sudo chown -R whrl:whrl ~/indigo_ws

RUN git clone https://github.com/ihmcrobotics/ihmc-open-robotics-software.git -b val-develop --single-branch ~/ihmc-open-robotics-software
# RUN cd ~/ihmc-open-robotics-software && git checkout val-develop 
RUN cd ~/ihmc-open-robotics-software && git clone https://github.com/ihmcrobotics/ihmc-path-planning.git 
RUN cd ~/ihmc-open-robotics-software/ihmc-path-planning && git checkout 0.10.0
RUN cd ~/ihmc-open-robotics-software && git clone https://github.com/ihmcrobotics/robot-environment-awareness.git
RUN cd ~/ihmc-open-robotics-software/robot-environment-awareness && git checkout 0.10.0
RUN cd ~/ihmc-open-robotics-software && ./gradlew && cd valkyrie && ../gradlew deployLocal
RUN echo 'export IHMC_SOURCE_LOCATION=~/ihmc-open-robotics-software' >> ~/.bashrc

# run warmup gradle to download most of the files required for ihmc controllers
RUN export IHMC_SOURCE_LOCATION=~/ihmc-open-robotics-software && \
		  source ~/indigo_ws/devel/setup.bash && \
                  roslaunch ihmc_valkyrie_ros valkyrie_warmup_gradle_cache.launch use_local_build:=true

ADD valkyrie_sim_gazebo_sync.urdf /opt/nasa/indigo/share/val_description/model/urdf
# This might not be required anymore
EXPOSE 8080
EXPOSE 8000 
EXPOSE 11311
EXPOSE 11611
EXPOSE 11711
EXPOSE 5900

# Run command that should be the entry poitn to our code
CMD /bin/bash -c "source ~/.bashrc && roslaunch srcsim finals.launch grasping_init:=false use_local_build:=true"
