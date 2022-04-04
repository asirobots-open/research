FROM asi_ros2_sim
COPY . /ros2_ws/src
WORKDIR /ros2_ws
RUN apt-get update
RUN apt-get install -y ros-galactic-rqt*
CMD rqt
