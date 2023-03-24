FROM ros:noetic

# Set the working directory to the home directory
WORKDIR /root

RUN apt-get update

RUN apt-get update && apt-get install -y gazebo11 libgazebo11-dev

ENV GAZEBO_MODEL_PATH=/path/to/gazebo/models:$GAZEBO_MODEL_PATH
ENV GAZEBO_PLUGIN_PATH=/path/to/gazebo/plugins:$GAZEBO_PLUGIN_PATH

RUN apt-get install ros-noetic-ackermann-msgs ros-noetic-geometry2 \
    ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-velodyne-simulator

RUN mkdir -p /root/gem_ws/src
RUN cd /root/gem_ws/src
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git
RUN git clone https://github.com/rikhsitlladeveloper/Path_tracking_controller.git

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/gem_ws && \
    catkin_make"

RUN echo "source /root/gem_ws/devel/setup.bash" >> /root/.bashrc

CMD ["roslaunch", "gem_gazebo", "gem_gazebo_rviz.launch"]

CMD ["roslaunch", "path_tracking_controller", "path_tracking.launch"]

