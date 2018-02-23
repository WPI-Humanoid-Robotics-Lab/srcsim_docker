# srcsim_docker
Docker images for srcsim

### Instructions
1. Install [docker-ce](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
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
2. Create a network bridge named srcsim. 
```bash
docker network create --subnet 192.168.1.0/16 --driver bridge srcsim
```
3. Install [nvidia-docker plugin](https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-1.0)). Please refer to prerequisites in that link.  
```bash
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  	
sudo apt-key add -
	
curl -s -L https://nvidia.github.io/nvidia-docker/ubuntu16.04/amd64/nvidia-docker.list | \
  	
sudo tee /etc/apt/sources.list.d/nvidia-docker.list
	
sudo apt-get update
	
sudo apt-get install nvidia-docker nvidia-modprobe
```

4. Run the script to pull docker image and run the container. Docker needs sudo access by default. If you see an error while running the docker image, use sudo. Refer to [this](https://askubuntu.com/questions/477551/how-can-i-use-docker-without-sudo) if you wish to allow docker commands without sudo.
```bash
# clone the repository if you have not done that already
git clone https://github.com/WPI-Humanoid-Robotics-Lab/srcsim_docker.git  -b master --single-branch
	
# Go the srcsim_docker directory
cd srcsim_docker/
	
# When running it for the first time, it might take a while depending on your internet speed
# The scripts have IHMC controller version number in the name
bash run_srcsim_docker0.9.bash
```

5. Connect gazebo client on host machine (in a new terminal)
```bash
GAZEBO_MASTER_URI=http://192.168.2.10:11345 gzclient
```
6. To run code on docker image export the following variables. You can add these to ~/.bashrc for ease of use.
```bash
export ROS_MASTER_URI=http://192.168.2.10:11311
export ROS_IP=192.168.0.1 # Confirm this from ifconfig results
```
7. Source ~/.bashrc and test connection
```bash
source ~/.bashrc
rosrun tough_controller_interface test_pelvis 0.8
```
8. To stop the docker run `docker stop srcsim`
