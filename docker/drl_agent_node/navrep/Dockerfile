FROM arena3d:latest

SHELL ["/bin/bash","-c"]

WORKDIR /root

#   Install required python version
RUN pyenv install 3.6.2

#   Set python to 3.6.2
RUN pyenv global 3.6.2 

RUN pip3 install --upgrade pip

ADD requirements.txt /root
RUN pip3 install -r requirements.txt

#   Clone and install navrep
RUN git clone https://github.com/ethz-asl/navrep.git
WORKDIR /root/navrep
RUN pip3 install -e .
RUN pip3 install gym==0.19.0

WORKDIR /root/src/arena_local_planner_drl

RUN cat /root/.bashrc
CMD source /opt/ros/noetic/setup.sh && \
    source /root/devel/setup.sh && \
    export PYTHONPATH=/root/devel/lib/python3/dist-packages:${PYTHONPATH} && \
    python scripts/deployment/drl_agent_node.py

