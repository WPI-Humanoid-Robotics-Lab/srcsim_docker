FROM nvidia/cuda:10.1-runtime-ubuntu14.04
# FROM ros:indigo-ros-base
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
RUN sudo apt-get -y update && sudo apt-get install -y wget software-properties-common
# Add ppa for Java
RUN sudo add-apt-repository ppa:openjdk-r/ppa

# setup ROS and gazebo 7 keys
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN /bin/bash -c "echo 'source /opt/ros/indigo/setup.bash' >> ~/.bashrc"
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget -O - http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -

# Install Gazebo7
RUN sudo apt-get -y update && sudo apt-get install -y ros-indigo-ros-base ros-indigo-gazebo7-ros-pkgs

# Copy NASA workspace for valkyrie
COPY nasa.tar.gz /tmp/nasa.tar.gz
RUN sudo tar -xvzf /tmp/nasa.tar.gz -C /opt/

# Install Dependencies 
RUN  sudo apt-get -y update && sudo apt-get install -y git \
  g++ vim nano wget  ca-certificates  ssh ruby ros-indigo-pcl-ros \
  x11vnc xvfb icewm lxpanel iperf xz-utils cmake screen terminator \ 
  ros-indigo-pcl-conversions ros-indigo-moveit \
  ros-indigo-trac-ik ros-indigo-footstep-planner \
  ros-indigo-humanoid-localization ros-indigo-multisense-ros \
  ros-indigo-laser-assembler ros-indigo-robot-self-filter \
  ros-indigo-tf2-geometry-msgs ros-indigo-joint-state-publisher \
  ros-indigo-octomap-server ros-indigo-octomap \
  ros-indigo-joint-trajectory-controller ros-indigo-robot-state-publisher \
  ros-indigo-image-transport \
  ros-indigo-joint-state-controller ros-indigo-position-controllers \
  ros-indigo-multimaster-fkie ros-indigo-effort-controllers \
  ros-indigo-moveit-full ros-indigo-sbpl \
  ros-indigo-humanoid-nav-msgs ros-indigo-map-server ros-indigo-trac-ik* \
  ros-indigo-multisense-ros ros-indigo-robot-self-filter ros-indigo-octomap \
  ros-indigo-octomap-msgs ros-indigo-octomap-ros ros-indigo-gridmap-2d \
  python-software-properties debconf-i18n openjdk-8-jdk mercurial \
  python-vcstool python-catkin-tools libxmlrpc-c++8-dev ros-indigo-orocos-* \
  ros-indigo-rtt-ros-integration ros-indigo-ros-control ros-indigo-stereo-image-proc \
  bison flex gperf libasound2-dev libgl1-mesa-dev \
  libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libjpeg-dev \
  libpng-dev libx11-dev libxml2-dev libxslt1-dev libxt-dev \
  libxxf86vm-dev pkg-config x11proto-core-dev \
  x11proto-xf86vidmode-dev libavcodec-dev mercurial \
  libgtk2.0-dev libgtk-3-dev \
  libxtst-dev libudev-dev libavformat-dev

RUN sudo rosdep init
RUN rosdep update 

# Setup java home
RUN sudo ln -s /usr/lib/jvm/java-8-openjdk-amd64 /usr/lib/jvm/default-java
ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64


RUN mkdir -p ${HOME}/.ihmc; curl https://raw.githubusercontent.com/ihmcrobotics/ihmc_ros_core/0.8.0/ihmc_ros_common/configurations/IHMCNetworkParametersTemplate.ini > ${HOME}/.ihmc/IHMCNetworkParameters.ini

# Set ulimit for realtime priority
RUN sudo bash -c 'echo "@ros - rtprio 99" > /etc/security/limits.d/ros-rtprio.conf'
RUN sudo groupadd ros
RUN sudo usermod -a -G ros whrl

# Download Gazebo models, so the simulation starts faster
RUN wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
RUN mkdir -p $HOME/.gazebo/models
RUN tar -xvf /tmp/default.tar.gz -C $HOME/.gazebo/models --strip 1
RUN rm /tmp/default.tar.gz


RUN /bin/bash -c "source /opt/nasa/indigo/setup.bash && mkdir ~/indigo_ws "
RUN /bin/bash -c "cd ~/indigo_ws && catkin config --init --mkdirs"
# RUN /bin/bash -c "wget https://raw.githubusercontent.com/WPI-Humanoid-Robotics-Lab/atlas_workspace/master/atlas_ws_0.9.yaml -O ~/indigo_ws/src/atlas_ws.yaml"
COPY valkyrie_ws_0.8.2.yaml /home/whrl/indigo_ws/src/
RUN cat /home/whrl/indigo_ws/src/valkyrie_ws_0.8.2.yaml
RUN /bin/bash -c "cd ~/indigo_ws/src && vcs import < valkyrie_ws_0.8.2.yaml"
RUN /bin/bash -c "cd ~/indigo_ws && rosdep install --from-paths src --ignore-src -r -y"

# Clone srcsim locally
COPY srcsim.tar.gz /tmp/srcsim.tar.gz
RUN tar -xvzf /tmp/srcsim.tar.gz -C ~/indigo_ws/src/ 

### Version specific
# can this be compiled locally? No. it needs javafx. we should find a solution to that for ubuntu 14.04 first
RUN cd && git clone https://github.com/ihmcrobotics/repository-group.git
RUN cd ~/repository-group && git clone https://github.com/WPI-Humanoid-Robotics-Lab/ihmc-open-robotics-software.git
RUN cd ~/repository-group/ihmc-open-robotics-software && git checkout 0.8.2-fix 
RUN sudo update-ca-certificates -f
#compile openfx locally
RUN /bin/bash -c "cd  && wget https://services.gradle.org/distributions/gradle-4.8-bin.zip \
  && unzip gradle-4.8-bin.zip"
ENV PATH ~/gradle-4.8/bin:${PATH}
COPY javafx-sdk-overlay.zip $JAVA_HOME
RUN /bin/bash -c "cd $JAVA_HOME && sudo unzip javafx-sdk-overlay.zip"
RUN cd ~/repository-group/ihmc-open-robotics-software/Valkyrie && \
  pwd && gradle tasks && gradle -q deployLocal


### /Version specific

# Compile the code
RUN /bin/bash -c "source /opt/nasa/indigo/setup.bash && cd ~/indigo_ws && sudo rm -rf build devel && catkin_make"
RUN sudo chown -R whrl:whrl ~/indigo_ws

# Nvidia stuff
RUN sudo bash -c "touch /etc/ld.so.conf.d/nvidia.conf"

RUN sudo bash -c 'echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf'
RUN sudo bash -c 'echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf'
ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility

# @todo: parameterize the version of nvidia
#nvidia-384 installs lightdm that requires a keyboard input. To avoid that popup, copy an existing keyboard layout file
COPY ./keyboard /etc/default/keyboard
RUN sudo mkdir -p /usr/lib/nvidia/
# RUN curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
# RUN sudo apt-get -y update && sudo DEBIAN_FRONTEND=noninteractive apt-get -y install nvidia-384

# setup bashrc
ARG ip
ENV IP=$ip
RUN echo "IP is ${IP}"
RUN /bin/bash -c "echo 'export ROS_MASTER_URI=http://${IP}:11311' >> ~/.bashrc"
RUN /bin/bash -c "echo 'export ROS_IP=${IP}' >> ~/.bashrc"                 
RUN echo 'export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64' >> ~/.bashrc
RUN echo 'export IHMC_SOURCE_LOCATION=$HOME/repository-group/ihmc-open-robotics-software'>> ~/.bashrc
RUN echo 'export IS_GAZEBO=true' >> ~/.bashrc

RUN echo 'source ~/indigo_ws/devel/setup.bash' >> ~/.bashrc
RUN echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
RUN source ~/.bashrc

# Run command that should be the entry poitn to our code
CMD /bin/bash 
#-c "source ~/.bashrc && roslaunch srcsim finals.launch grasping_init:=false"
