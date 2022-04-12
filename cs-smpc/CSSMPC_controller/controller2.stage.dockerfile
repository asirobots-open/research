# This is the dockerfile that is used as the image for the controller. Move it to whereever
# you need to in order for it to contain your controller

# THIS "#VY CONTEXT" LINE MEANS ANYTHING THAT NEEDS TO BE COPIED INTO THE CONTAINER
# SHOULD LIVE UNDER THE SAME DIRECTORY AS THIS dockerfile
#VY CONTEXT .

# START FROM THE asi_boring_sim IMAGE
# FROM asi_boring_sim
FROM asirobots/asi_ros2_sim:foxy

# INSTALL Dependencies
# RUN wget https://github.com/deadsnakes/python3.6/archive/refs/tags/debian/3.6.13-1+xenial1.tar.gz
# RUN tar -xvf 3.6.13-1+xenial1.tar.gz
# RUN /bin/sh python3.6-debian-3.6.13-1-xenial1/configure
# RUN make
# RUN make install
# RUN python3.6 -V
RUN python3 -V
RUN pip3 -V
# RUN apt-get install -y python3-pip
# RUN python3.6 -m pip install --upgrade pip
RUN pip3 install PyYAML rospkg catkin_pkg
RUN pip3 install numpy --user
RUN pip3 install scipy --user
RUN pip3 install Mosek --user
RUN pip3 install transforms3d
#RUN pip3 install -r requirements.txt

COPY --from=asi_boring_sim /ros2_ws /ros2_ws

# COPY YOUR CODE INTO THE IMAGE
# COPY . .
COPY . /ros2_ws/src

WORKDIR /ros2_ws

RUN ls src/
RUN lsb_release -a

# RUN cp -r asi_msgs/ src/
RUN /bin/bash -c '. /opt/ros/$ROS2/setup.bash; colcon build --packages-select asi_msgs asi_ctrl --merge-install'
RUN mkdir /root/mosek
RUN cp src/mosek.lic /root/mosek

# COMPILE
# RUN bash -c ". /opt/ros/$ROS2/setup.bash && \
#             . install/setup.bash && \
#             rosdep install -i --from-path src --rosdistro $ROS2 -y \
#             && colcon build --packages-select boring_nodes"


# PUT THE RUN COMMAND HERE OR IN THE COMPOSE FILE
CMD bash -c ". until rostopic list ; do sleep 1; done && sleep 3 && python cs_controller_asi_convex.py"

#VY SOURCE asirobots/asi_ros2_sim:foxy