# Woking with Docker
Using Docker, you can run this repo on any operating system. Please follow these steps to install arena-rosnav-3D via Docker:
#### Install latest version of docker


- For Ubuntu 20.04 (src: https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-20-04)
```bash
sudo apt update
sudo apt install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io
docker -v 
# Docker version 20.10.7, build f0df350
```
- For Windows
https://docs.docker.com/desktop/windows/install/

- For Mac
https://docs.docker.com/desktop/mac/install/


#### Executing Docker without sudo (optional but recommended)
```bash
sudo usermod -aG docker ${USER}
su - ${USER}
```

## Build the Docker
1. go to the folder ../arena-rosnav-3D/docker
```
cd arena-rosnav-3D/docker
```
3. build the docker images 
```
sudo docker build --no-cache -t v1 .
```

# Using the Docker
## Without Visualization
Run the docker image name-of-the-docker-image (here: v1)
```
sudo docker run -it --rm --net=host v1
```

## With Visualization 
### On Native Ubuntu
```
xhost +
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix v1
```
Do the quickstart
```
roslaunch arena_bringup start_arena_gazebo.launch local_planner:=teb task_mode:=random world:=small_warehouse actors:=6 
```
### On Windows and Mac using docker-compose 
Install docker-compose

```
sudo curl -L "https://github.com/docker/compose/releases/download/1.27.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose \
sudo chmod +x /usr/local/bin/docker-compose \
docker-compose --version

```
Using docker-compose, you can set up a configuration.yml file where you have to setup display variables. A [docker-compose.yml](https://github.com/ignc-research/arena-rosnav-3D/docker/docker-compose.yml)configuration file is already given in this repo. You might have to modify it.

```
docker-compose up -d \
docker-compose start \
```
Enter the contaier
```
docker exec -it v1:v1 bash
```
Do the quickstart
```
roslaunch arena_bringup start_arena_gazebo.launch local_planner:=teb task_mode:=random world:=small_warehouse actors:=6 
```
Then, open http://localhost:8080/vnc.html you will see the rviz window in browser
