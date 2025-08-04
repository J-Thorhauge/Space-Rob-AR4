FROM ros:humble

RUN git clone https://github.com/J-Thorhauge/Space-Rob-AR4.git

RUN cd Space-Rob-AR4/

RUN apt update
RUN rosdep update
RUN rosdep install --from-paths . --ignore-src -r -y

RUN colcon build
RUN source install/setup.bash



