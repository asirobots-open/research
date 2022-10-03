FROM ubuntu:bionic
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y \
  build-essential \
  python3-dev \
  python3-pip \
  wget \
  curl \
  lsb-release \
  locales \
  libcurl4-openssl-dev \
  libssl-dev \
  openssl

# Need to upgrade cmake
ADD https://github.com/Kitware/CMake/releases/download/v3.24.0/cmake-3.24.0.tar.gz /cmake-3.24.0.tar.gz
RUN mkdir /opt/cmake
RUN tar xf /cmake-3.24.0.tar.gz
RUN cd /cmake-3.24.0 && ./bootstrap --prefix=/opt/cmake && make && make install
RUN ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake

RUN apt-get update && apt install -y valgrind gdb
