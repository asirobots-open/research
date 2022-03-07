FROM asirobots/asi_ros2_sim:0.0.2
# FROM asirobots-car_sim_ros2

COPY ros2/src /ros2_ws/src
WORKDIR /ros2_ws
RUN bash -c ". /opt/ros/$ROS2/setup.bash && \
            . install/setup.bash && \
            rosdep install -i --from-path src --rosdistro $ROS2 -y \
            && colcon build --packages-select boring_nodes"

#VY CONTEXT ..
#VY SOURCE asirobots/asi_ros2_sim:0.0.2
