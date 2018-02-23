# srcsim_docker
Docker images for srcsim

### Instructions
1. Install [docker-ce](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
2. create a network bridge named srcsim
	```bash
	docker network create --subnet 192.168.1.0/16 --driver bridge srcsim
	```
3. Install nvidia-docker plugin `https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-1.0)` 
4. Run the script to pull docker image and run the container
	```bash
	bash run_srcsim_docker0.9.bash
	```
5. Connect gazebo client on host machine (in a new terminal)
	```bash
	GAZEBO_MASTER_URI=http://192.168.2.10:11345 gzclient
	```
6. To run code on docker image add the following 2 variables to ~/.bashrc
	```bash
	export ROS_MASTER_URI=http://192.168.2.10:11311
	export ROS_IP=192.168.0.1 # Confirm this from ifconfig results
	```
7. Source ~/.bashrc and test connection
       ```bash
       source ~/.bashrc
       rosrun tough_controller_interface test_pelvis 0.8
       ```
