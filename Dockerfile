FROM ros:humble

RUN apt-get update && apt-get install -y nano && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /app && chmod -R 755 /app
WORKDIR /app


ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd -s /bin/bash --uid 1000 --gid ${USER_GID} -m ${USERNAME} \
  && mkdir /home/${USERNAME}/.config && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config

RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout ${USERNAME}

# USER ros

RUN git clone -b Docker-camera https://github.com/J-Thorhauge/Space-Rob-AR4.git

RUN cd Space-Rob-AR4/

# USER root

RUN apt update

USER ros

RUN rosdep update
RUN rosdep install --from-paths . --ignore-src -r -y

USER root

RUN apt-get update \
  && apt-get install -y \
  libserial-dev \
  sensor_msgs \
  cv_bridge \
  OpenCV \
  && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash"
# RUN colcon build
# RUN source install/setup.bash

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && bash"]

