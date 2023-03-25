FROM ros:noetic

# Set the working directory to the home directory

RUN apt-get update

RUN apt-get update && apt-get install -y gazebo11 libgazebo11-dev

ENV GAZEBO_MODEL_PATH=/path/to/gazebo/models:$GAZEBO_MODEL_PATH
ENV GAZEBO_PLUGIN_PATH=/path/to/gazebo/plugins:$GAZEBO_PLUGIN_PATH
ENV DEBIAN_FRONTEND=noninteractive
ENV PATH=/opt/ros/noetic/bin:$PATH
RUN apt-get install -y ros-noetic-ackermann-msgs ros-noetic-geometry2 \
    ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-velodyne-simulator
RUN apt-get update && apt-get install -y git
RUN mkdir -p /gem_ws/src
WORKDIR /gem_ws
RUN cd /gem_ws/src && \
     /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"
RUN cd /gem_ws/src && \
    git clone https://github.com/rikhsitlladeveloper/Path_tracking_controller.git && \
    git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git

RUN cd /gem_ws && \
    rosdep install --from-paths src --ignore-src -r -y

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; \
                  catkin_make"


RUN echo "source /gem_ws/devel/setup.bash" >> /root/.bashrc
