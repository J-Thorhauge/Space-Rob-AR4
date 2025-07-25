FROM ros:humble

RUN apt-get update && apt-get install -y nano && rm -rf /var/lib/apt/lists/*

RUN git clone -b Docker https://github.com/J-Thorhauge/Space-Rob-AR4.git

RUN cd Space-Rob-AR4/


ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid &USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

USER ros

RUN apt update
RUN rosdep update
RUN rosdep install --from-paths . --ignore-src -r -y

RUN colcon build
RUN source install/setup.bash

USER root

