FROM ubuntu:16.04

RUN apt update
RUN apt install -y cmake git

# https://stackoverflow.com/a/73005414
RUN echo "#!/bin/bash\n\$@" > /usr/bin/sudo
RUN chmod +x /usr/bin/sudo

RUN apt-get install -y build-essential \
  cmake \
  curl \
  gcc \
  git \
  libglew-dev \
  libgtk2.0-dev \
  pkg-config \
  libavcodec-dev \
  libavformat-dev \
  libswscale-dev \
  python-dev \
  python-numpy \
  unzip

# build-essential
# cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
# python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
# curl
# unzip

ENV EIGEN_VERSION 3.3.2
ENV EIGEN_DOWNLOAD_URL http://bitbucket.org/eigen/eigen/get/$EIGEN_VERSION.tar.gz

RUN curl -fsSL https://github.com/opencv/opencv/archive/2.4.13.zip -o 2.4.13.zip
RUN unzip 2.4.13.zip
RUN rm 2.4.13.zip
WORKDIR /opencv-2.4.13
RUN mkdir build
WORKDIR /opencv-2.4.13/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
RUN make -j8
RUN make install

WORKDIR /
RUN curl -fsSL https://gitlab.com/libeigen/eigen/-/archive/3.2.10/eigen-3.2.10.tar.gz -o eigen-3.2.10.tar.gz
RUN mkdir /usr/include/eigen
RUN tar -xf eigen-3.2.10.tar.gz --strip-components=1 -C /usr/include/eigen
RUN rm eigen-3.2.10.tar.gz

RUN git clone --recursive --branch v0.5 https://github.com/stevenlovegrove/Pangolin.git
WORKDIR /Pangolin
RUN mkdir build
WORKDIR /Pangolin/build
RUN cmake ..
RUN make -j8

WORKDIR /
RUN git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
WORKDIR /ORB_SLAM2
# https://github.com/raulmur/ORB_SLAM2/pull/144
RUN sed -i '25i#include <unistd.h>' include/System.h
RUN sed -i '31s/$/1/' build.sh
RUN ./build.sh

# RUN curl -fsSL https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_teddy.tgz -o rdbd_dataset_freiburg1_teddy.tgz
# RUN tar -xvf rdbd_dataset_freiburg1_teddy.tgz

# ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml rgbd_dataset_freiburg1_teddy/
