FROM arena3d:latest

SHELL ["/bin/bash","-c"]

WORKDIR /root

ADD requirements.txt /root

#   Install required python version and se it as global
#   Install requirements
RUN pyenv install 3.6.2 \
    && pyenv global 3.6.2 \
    && pip3 install --upgrade pip \
    && pip3 install -r requirements.txt    

#   Clone and install navrep
# RUN git clone https://github.com/danieldugas/drl_local_planner_ros_stable_baselines.git /root/guldenring_repo
RUN git clone https://github.com/sudo-Boris/drl_local_planner_ros_stable_baselines /root/guldenring_repo
RUN mkdir /root/guldenring \
    && mv /root/guldenring_repo/rl_agent /root/guldenring/rl_agent \
    && rm -r /root/guldenring_repo
WORKDIR /root/guldenring/rl_agent
RUN pip3 install -e . \
    && pip3 install gym==0.19.0

WORKDIR /root/src/arena_local_planner_drl
CMD source /opt/ros/noetic/setup.sh && \
    source /root/devel/setup.sh && \
    export PYTHONPATH=/root/devel/lib/python3/dist-packages:${PYTHONPATH} && \
    python scripts/deployment/drl_agent_node.py

