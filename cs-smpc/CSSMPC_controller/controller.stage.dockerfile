# This is the dockerfile that is used as the image for the controller. Move it to whereever
# you need to in order for it to contain your controller

# THIS "#VY CONTEXT" LINE MEANS ANYTHING THAT NEEDS TO BE COPIED INTO THE CONTAINER
# SHOULD LIVE UNDER THE SAME DIRECTORY AS THIS dockerfile
#VY CONTEXT .

# START FROM THE asi_boring_sim IMAGE
FROM asi_boring_sim  

# COPY YOUR CODE INTO THE IMAGE
COPY . .

# INSTALL ANYTHING YOU MIGHT NEED
# RUN apt install -y vim
#RUN apt-get update
# RUN apt-get install -y software-properties-common
# #RUN add-apt-repository "deb http://archive.ubuntu.com/ubuntu $(lsb_release -sc) universe"
# RUN add-apt-repository ppa:deadsnakes/ppa
# RUN apt-get update
# RUN apt-get install -y python3.9
#RUN apt-get install -y python3-matplotlib
#RUN apt-get install -y python3-numpy
#RUN apt-get install -y python3-scipy

RUN wget https://github.com/deadsnakes/python3.6/archive/refs/tags/debian/3.6.13-1+xenial1.tar.gz
RUN tar -xvf 3.6.13-1+xenial1.tar.gz
RUN /bin/sh python3.6-debian-3.6.13-1-xenial1/configure
RUN make
RUN make install
RUN python3.6 -V

RUN apt-get install -y python3-pip
RUN python3.6 -m pip install --upgrade pip
RUN pip3.6 install PyYAML rospkg catkin_pkg
RUN pip3.6 install numpy --user
RUN pip3.6 install scipy --user
RUN pip3.6 install Mosek --user
#RUN pip3 install -r requirements.txt

RUN cp -r asi_msgs/ src/
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_make'
RUN mkdir /root/mosek
RUN cp mosek.lic /root/mosek

# COMPILE


# PUT THE RUN COMMAND HERE OR IN THE COMPOSE FILE
CMD bash -c ". install/setup.bash && until rostopic list ; do sleep 1; done && sleep 3 && python cs_controller_asi_convex.py"

