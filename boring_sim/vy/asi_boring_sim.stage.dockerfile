FROM asirobots/asi_ros2_sim:foxy
# FROM asirobots-car_sim_ros2
RUN pip3 install transforms3d
RUN apt-get update
RUN apt-get install -y ros-foxy-rqt*
COPY src /ros2_ws/src
WORKDIR /ros2_ws
RUN bash -c ". /opt/ros/$ROS2/setup.bash && \
            . install/setup.bash && \
            rosdep install -i --from-path src --rosdistro $ROS2 -y \
            && colcon build --merge-install --packages-select boring_nodes"

#VY CONTEXT ..
#VY SOURCE asirobots/asi_ros2_sim:foxy
