# DRL agent node Docker

Dockerfiles in this directory can be used to run the drl agent node. In this node the model is invoked and a new _cmd_vel_ is predicted based on the current observation.

## Supported Dockers

Currently we have Dockerfiles for running models trained in either [navrep](#TODO), [guldenring](#TODO) or [rosnav](#TODO).

## Usage

# TODO

docker build -t arena3d -f ./docker/drl_agent_node/Dockerfile .

docker run -ti --network host navrep

docker run -ti -v /home/reyk/Schreibtisch/Uni/VIS/arena-rosnav-3d/src/arena-rosnav-3D/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl:/root/src/arena_local_planner_drl --network host navrep

## Why Docker?

You may wonder, why we have to use a seperate docker container for each model. The reason is, that because every model is trained in different environment we have to use different python and package versions. Docker is the fastest way to implement this.
