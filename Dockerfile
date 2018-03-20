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

# Install srcsim and Gazebo7
RUN /bin/bash -c "echo 'source /opt/ros/indigo/setup.bash' >> ~/.bashrc"
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget -O - http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
RUN sudo sh -c 'echo "deb http://srcsim.gazebosim.org/src trusty main" > /etc/apt/sources.list.d/src-latest.list'
RUN wget -O - http://srcsim.gazebosim.org/src/src.key | sudo apt-key add -
RUN wget -O - https://bintray.com/user/downloadSubjectPublicKey?username=bintray | sudo apt-key add -
RUN sudo apt-get -y update && sudo apt-get install -y srcsim

# Install Dependencies 
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

RUN wget -P /tmp/ https://osrf-distributions.s3.amazonaws.com/srcsim/valkyrie_controller.0.8.2.tar.gz 
RUN tar -xvf /tmp/valkyrie_controller.0.8.2.tar.gz -C $HOME
RUN rm /tmp/valkyrie_controller.0.8.2.tar.gz

RUN /bin/bash -c "echo 'export ROS_MASTER_URI=http://201.1.1.10:11311' >> ~/.bashrc"
RUN /bin/bash -c "echo 'export ROS_IP=201.1.1.10' >> ~/.bashrc"                 
RUN /bin/bash -c "source ~/.bashrc"

RUN wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
RUN mkdir -p $HOME/.gazebo/models
RUN tar -xvf /tmp/default.tar.gz -C $HOME/.gazebo/models --strip 1
RUN rm /tmp/default.tar.gz

RUN source /opt/nasa/indigo/setup.bash

# Clone additional repos that are required for our code
#RUN git clone https://github.com/ninja777/humanoid_navigation.git ~/indigo_ws/src/humanoid_navigation
#RUN cd ~/indigo_ws/src/humanoid_navigation && git checkout indigo-devel
RUN mkdir ~/indigo_ws/src/ihmc_repos
RUN git clone https://github.com/WPI-Humanoid-Robotics-Lab/ihmc_ros_core.git ~/indigo_ws/src/ihmc_repos/ihmc_ros_core
RUN cd ~/indigo_ws/src/ihmc_repos/ihmc_ros_core && git fetch && git checkout warmupfix-0.8.2
RUN git clone https://github.com/ihmcrobotics/ihmc_valkyrie_ros.git ~/indigo_ws/src/ihmc_repos/ihmc_valkyrie_ros
RUN cd ~/indigo_ws/src/ihmc_repos/ihmc_valkyrie_ros && git checkout 0.8.2
RUN git clone https://github.com/ihmcrobotics/ihmc-ros-control.git ~/indigo_ws/src/ihmc_repos/ihmc_ros_control
RUN cd ~/indigo_ws/src/ihmc_repos/ihmc_ros_control && git checkout 0.5.0

RUN /bin/bash -c "source ~/indigo_ws/devel/setup.bash && \
                  roslaunch ihmc_valkyrie_ros valkyrie_warmup_gradle_cache.launch"

# Compile the code
RUN /bin/bash -c "source ~/.bashrc && cd ~/indigo_ws && sudo rm -rf build devel && catkin_make"
RUN sudo chown -R whrl:whrl ~/indigo_ws

ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

# Run command that should be the entry poitn to our code
CMD /bin/bash -c "source ~/.bashrc && roslaunch srcsim finals.launch"
