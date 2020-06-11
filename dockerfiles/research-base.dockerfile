FROM ubuntu:bionic
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  python3-dev \
  python3-pip \
  wget \
  curl \
  lsb-release \
  locales