# srcsim_docker
Docker images for srcsim

### Instructions
1. Install [docker-ce](https://docs.docker.com/install/linux/docker-ce/ubuntu/) using the instructions given below. If docker is already installed upgrade it to 19.03.x or greater and skip this step.  
```bash
sudo apt-get update

# to uninstall older versions of docker
sudo apt-get remove docker docker-engine docker.io

sudo apt-get install \
     linux-image-extra-$(uname -r) \
     linux-image-extra-virtual
	
sudo apt-get update
	
sudo apt-get install \
     apt-transport-https \
     ca-certificates \
     curl \
     software-properties-common
 	
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
 	
sudo apt-key fingerprint 0EBFCD88
#Verify that you now have the key with the fingerprint 9DC8 5822 9FC7 DD38 854A E2D8 8D81 803C 0EBF CD88, by searching 	  	   the last 8 characters of the fingerprint.
 	
sudo add-apt-repository \
     "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
     $(lsb_release -cs) \
     stable"
	
sudo apt-get update
	
sudo apt-get install docker-ce
```
2. Install [nvidia-docker plugin](https://github.com/NVIDIA/nvidia-docker). Please refer to prerequisites in that link.  
```bash
# Add the package repositories
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```
3. Check Nvidia driver version on host machine
```bash
# find out the version of nvidia drivers installed
glxinfo | grep "OpenGL core profile version" 
# output of this should be something similar to 
# OpenGL core profile version string: 4.5.0 NVIDIA 384.130
# In this case, the driver version is 384.130

# search for the bin, lib and lib32 directories
dpkg -L nvidia-384 # change nvidia-384 to the major number of installed version
# based on the output, relevant directories on host machine are /usr/lib/nvidia-384/bin, /usr/lib/nvidia-384, and /usr/lib32/nvidia-384

```

4. Open ~/.bashrc file and add the following lines to it. 

```bash
######## srcsim_docker ##############
export NVIDIA_BIN="/usr/lib/nvidia-384/bin"
export NVIDIA_LIB="/usr/lib/nvidia-384"
export NVIDIA_LIB32="/usr/lib32/nvidia-384"

######### docker aliases #############
DUID=$((UID%256))
export IP=${IPADDR:-172.16.$DUID.$DUID}
alias source_dock="export ROS_MASTER_URI=http://${IP}:11311 && \
                  export ROS_IP=172.16.$DUID.1" # Confirm this from ifconfig results

export SRCSIM_DOCKER_DIR="~/srcsim_docker" # change this based on your configuration
alias start_dock="cd $SRCSIM_DOCKER_DIR && bash run_srcsim_docker.bash"
alias stop_dock="docker stop srcsim_${USER}"
alias gazebo_dock="GAZEBO_MASTER_URI=http://${IP}:11345 gzclient"

######### ROS  ##########
source /opt/ros/kinetic/setup.bash
source ~/kinetic_ws/install/setup.bash
source ~/kinetic_ws/install/share/drcsim/setup.sh

############ IHMC Controllers ################
export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64/
export IHMC_SOURCE_LOCATION=~/repository-group/ihmc-open-robotics-software
```

5. Run the script to build docker image and run the container. Docker needs sudo access by default. If you see an error while running the docker image, use sudo. Refer to [this](https://askubuntu.com/questions/477551/how-can-i-use-docker-without-sudo) if you wish to allow docker commands without sudo.
```bash
# on host machine
source ~/.bashrc
# clone the repository if you have not done that already
git clone https://github.com/WPI-Humanoid-Robotics-Lab/srcsim_docker.git  -b 0.8.2
	
# Go the srcsim_docker directory
cd srcsim_docker/
	
# When running it for the first time, it might take a while depending on your internet speed
# The scripts have IHMC controller version number in the name
start_dock
```

7. The above command will create a docker container and provide bash shell. Type the following command in the shell. It will take about 30 minutes to download all the dependencies. You should see gazebo client with Valkyrie robot. 
```bash
# In docker shell
roslaunch srcsim unique_task1.launch grasping_init:=false use_local_build:=true
```
>The robot cannot be controlled in the first run. When the shell displays `IHMC ROS API node successfully started.
 Building 66% > :runJavaDelegate`, move to next step. 

8. Terminate the roslaunch command with `CTRL+C` and commit the changes to docker container.
```bash
# on host machine
docker commit srcsim_${USER} srcsim_0.8.2:${DUID}
```

9. Restart it with following command.
```bash
# In docker shell
roslaunch srcsim unique_task1.launch grasping_init_wait_time:=200 use_local_build:=true
# depending on the host machine configuration, the grasping_init_wait_time can change. It should be such that it allows for the robot to be standing before the grasping controllers initialize.
```
10. In a new terminal,  source ~/.bashrc and test connection
```bash
# on host machine
source ~/.bashrc
rosrun tough_controller_interface test_pelvis 0.8
```
11. To stop the docker run `stop_dock`.
