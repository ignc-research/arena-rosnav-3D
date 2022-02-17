FROM conventional_planner_3d:latest

SHELL ["/bin/bash","-c"]

WORKDIR /root

ADD ./docker/conventional_planners/cadrl/requirements.txt /root/requirements.txt

#   Install required python version and se it as global
#   Install requirements
RUN pyenv install 3.6-dev \
    && pyenv global 3.6-dev \
    && pip3 install --upgrade pip \
    && pip3 install -r requirements.txt \ 
    && pip3 uninstall -y enum34

#   add cadrl node
ADD ./arena_navigation/arena_local_planer/model_based/cadrl_ros ./src/cadrl_ros
WORKDIR /root/src
RUN git clone https://bitbucket.org/acl-swarm/ford_msgs/src/master ford_msgs
WORKDIR /root
RUN source /opt/ros/noetic/setup.sh \
    && catkin_make


CMD source /opt/ros/noetic/setup.sh && \
    source /root/devel/setup.sh && \
    export PYTHONPATH=/root/devel/lib/python3/dist-packages:${PYTHONPATH} && \
    roslaunch cadrl_ros cadrl_node.launch

