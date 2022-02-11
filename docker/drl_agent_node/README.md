# DRL agent node Docker

Dockerfiles in this directory can be used to run the drl agent node. In this node the model is invoked and a new _cmd_vel_ is predicted based on the current observation.

## Supported Dockers

Currently we have Dockerfiles for running models trained in either [navrep](#TODO), [guldenring](#TODO) or [rosnav](#TODO).

## Building Docker

### Automated building

We provide a shell skript for you to simplify the docker build process.
Just run:

```bash
cd ~/catkin_ws/src/arena-rosnav-3D/docker/drl_agent_node
chmod +x build_docker.sh && ./build_docker.sh
```
> **NOTE**: If this script returns: 'Install docker', install docker on your system first, see the documentation [here](https://github.com/ignc-research/arena-rosnav-3D/tree/main/docker#install-latest-version-of-docker). (You only need to install and setup docker, therefore, only follow the steps in the first section).

To start the build process.
When calling `build_docker.sh` with no arguments the script will build all docker containers in the `drl_agent_node` directory.
If you only want to build one container, for example the _navrep_ container, you can pass the directory as argument.
This command would only build the main docker and the _navrep_ docker:

```bash
chmod -x build_docker.sh && ./build_docker.sh navrep/
```

### Manual build

All docker containers are build on top the main dockerfile contained in this directory, which provides some packages all others will need. To build this docker run:

```bash
cd ../.. && docker build -t arena3d -f ./docker/drl_agent_node/Dockerfile . && cd -
```

Changing directory is needed to get the desired build context.

After building the main container you can start building the environment containers by changing directory in the desired folder and running:

```bash
docker build -t <name_of_your_container> .
```

## Usage

You can now start the docker by calling:

```bash
docker run -ti --network host <name_of_your_container>
# Example
docker run -ti --network host navrep
```

This will launch your container with the drl node files added at build time of the main container.

If you want to add the current state of the _arena_local_planner_drl_ directory you can call the docker with an additional volume:

```bash
docker run -ti -v /<absolute_path_to_diretory>/arena_local_planner_drl:/root/src/arena_local_planner_drl --network host <name_of_your_container>
```

This can be used while developing.

## Why Docker?

You may wonder, why we have to use a seperate docker container for each model. The reason is, that because every model is trained in different environment we have to use different python and package versions. Docker is the fastest way to implement this.
