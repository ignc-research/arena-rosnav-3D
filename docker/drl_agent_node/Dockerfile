FROM ros:noetic-ros-core-focal AS ros_base

SHELL ["/bin/bash","-c"]

RUN echo "source /opt/ros/noetic/setup.sh" >> /root/.bashrc

#   Install Python
RUN apt-get -y update \
    && apt-get install -y \
    apt-utils \
    software-properties-common \
    git \
    wget \
    ros-noetic-tf2 \
    ros-noetic-tf \
    ros-noetic-tf2-geometry-msgs \
    ffmpeg \
    libsm6 \
    libxext6  \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get -y update \
    && apt-get -y install \
    python3 \
    python3-pip

#   Install Poetry
RUN pip3 install poetry \
    && pip3 install --upgrade pip 

#   Install PyEnv
WORKDIR /root/
RUN git clone --depth=1 https://github.com/pyenv/pyenv.git .pyenv
ENV PYENV_ROOT="/root/.pyenv"
ENV PATH="${PYENV_ROOT}/shims:${PYENV_ROOT}/bin:${PATH}"


RUN echo 'eval "$(pyenv init -)"' >> /root/.bashrc
RUN sed -Ei -e '/^([^#]|$)/ {a export PYENV_ROOT="$HOME/.pyenv" \nexport PATH="$PYENV_ROOT/bin:$PATH"' -e ':a' -e '$!{n;ba};}' /root/.profile
RUN echo 'eval "$(pyenv init --path)"' >> /root/.profile

# Add the arena_local_planner_drl and catkin_make it
WORKDIR /root
ADD ./arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl ./src/arena_local_planner_drl
RUN source /opt/ros/noetic/setup.sh \
    && catkin_make
RUN echo "source /root/devel/setup.sh" >> /root/.bashrc
RUN echo "export PYTHONPATH=/root/devel/lib/python3/dist-packages:$PYTHONPATH" >> /root/.bashrc