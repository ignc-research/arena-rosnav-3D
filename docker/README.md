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
sudo docker build --no-cache -t rosnav-3d:v1 .
```


# Using the Docker
## Without Visualization
Run the docker image name-of-the-docker-image (here: rosnav-3d:v1,; v1 for version1)
```
sudo docker run -it --rm --net=host rosnav-3d:v1
```

## With Visualization 
### On Native Ubuntu
```
xhost +
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix rosnav-3d:v1 <local_planner> <task_mode> <world> <model> <actors> <recording>
```
If you are working with an NVIDIA graphics card, you may run into errors while using the docker with visualization. In this case, making sure that the same NVIDIA drivers are installed on both your Ubuntu and the Docker may resolve your problems.

Optional arguments: 

| Parameter                 | Default       | Options   |	
| :------------------------ |:-------------:| :-------------|
| local_planner 	       | teb           |`teb`, `dwa`, `mpc`, `cadrl (DRL)`, `rlca(DRL)`, `arena (DRL)`, `rosnav (DRL)`
| task_mode        | random          |`random`, `scenario`, `manual`, `staged` 
| world 	       |	small_warehouse	            |`small_warehouse`,`aws_house`,`turtlebot3_house`,`small_warehouse`,`random world`,`factory`,`hospital`,`experiment_rooms`,`bookstore`,`turtlebot3_world`
| model 		       | turtlebot3_burger	           | `turtlebot3_burger`, `ridgeback`, `jackal`, `agv-ota`
| actors 		           | 4             | `any integer, maximum of 40 recommended`
| recording 		           | false             | `bool: record a csv or not`

e.g.

```
 docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix rosnav-3D:v1 teb random aws_house 22 true    
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
docker exec -it rosnav-3d:v1 bash
```
Do the quickstart
```
roslaunch arena_bringup start_arena_gazebo.launch local_planner:=teb task_mode:=random world:=small_warehouse actors:=6 
```
Then, open http://localhost:8080/vnc.html you will see the rviz window in browser


# Using Docker Shell instead of directly running as a script
If you want to start docker in a shell and manually run the commands rather than using the Docker as a script, comment out the last two lines of the Dockerfile

```
# COPY /entrypoint_testing.sh /entrypoint_testing.sh
# ENTRYPOINT /entrypoint_testing.sh
``` 
and build the Docker again:
```
sudo docker build -t rosnav-3d:v1 .
```
### On Native Ubuntu
```
xhost +
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix rosnav-3d:v1
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
docker exec -it rosnav-3d:v1 bash
```
Do the quickstart
```
roslaunch arena_bringup start_arena_gazebo.launch local_planner:=teb task_mode:=random world:=small_warehouse actors:=6 
```
Then, open http://localhost:8080/vnc.html you will see the rviz window in browser


