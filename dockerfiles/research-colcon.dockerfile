FROM asirobots/research-base
RUN apt-get update && apt install -y cppcheck git
RUN pip3 install -U colcon-common-extensions
RUN mkdir -p /tmp/workspace/src
WORKDIR /tmp/workspace/src
RUN git clone https://github.com/ament/ament_package.git
RUN git clone https://github.com/ament/ament_cmake.git
RUN git clone https://github.com/ament/ament_lint.git
RUN git clone https://github.com/ament/googletest.git
WORKDIR /tmp/workspace
RUN colcon build --merge-install