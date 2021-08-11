# This is the dockerfile that is used as the image for the controller. Move it to whereever
# you need to in order for it to contain your controller

# THIS "#VY CONTEXT" LINE MEANS ANYTHING THAT NEEDS TO BE COPIED INTO THE CONTAINER
# SHOULD LIVE UNDER THE SAME DIRECTORY AS THIS dockerfile
#VY CONTEXT .

# START FROM THE asi_ros1_sim IMAGE
FROM asi_ros1_sim  

# INSTALL ANYTHING YOU MIGHT NEED
# RUN apt install -y vim

# COPY YOUR CODE INTO THE IMAGE
COPY . .


# COMPILE


# PUT THE RUN COMMAND HERE OR IN THE COMPOSE FILE
CMD bash -c ". install/setup.bash && until rostopic list ; do sleep 1; done && sleep 3 && python boring_controller.py"

