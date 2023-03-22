FROM ubuntu:16.04

RUN apt update
RUN apt install -y build-essential \
 cmake \
 curl \
 gcc \
 git \
 libglew-dev \
 libgtk2.0-dev \
 pkg-config \
 libavcodec-dev \
 libavformat-dev \
 libdc1394-22-dev \
 libjasper-dev \
 libjpeg-dev \
 libpng-dev \
 libswscale-dev \
 libtbb2 \
 libtbb-dev \
 libtiff-dev \
 python-dev \
 python-numpy \
 unzip


# https://stackoverflow.com/a/73005414
RUN echo "#!/bin/bash\n\$@" > /usr/bin/sudo
RUN chmod +x /usr/bin/sudo


RUN git clone --recursive --branch v0.5 https://github.com/stevenlovegrove/Pangolin.git
WORKDIR /Pangolin
RUN mkdir build
WORKDIR /Pangolin/build
RUN cmake ..
RUN make -j8


WORKDIR /
RUN curl -fsSL https://github.com/opencv/opencv/archive/2.4.13.zip -o 2.4.13.zip
RUN unzip 2.4.13.zip
RUN rm 2.4.13.zip
WORKDIR /opencv-2.4.13/
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


RUN apt install -y lsb-release
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt update
RUN apt install -y ros-kinetic-desktop
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
RUN rosdep init
RUN rosdep update


WORKDIR /
RUN git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
WORKDIR /ORB_SLAM2
RUN sed -i '31s/$/8/' build.sh
RUN ./build.sh


RUN ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen
RUN sed -i '43ifind_package(Boost COMPONENTS system)' Examples/ROS/ORB_SLAM2/CMakeLists.txt
RUN sed -i '50i\${Boost_INCLUDE_DIRS}' Examples/ROS/ORB_SLAM2/CMakeLists.txt
RUN sed -i '57i\${Boost_LIBRARIES}' Examples/ROS/ORB_SLAM2/CMakeLists.txt
RUN . /opt/ros/kinetic/setup.sh && echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/ORB_SLAM2/Examples/ROS" >> ~/.bashrc
RUN . /opt/ros/kinetic/setup.sh && export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/ORB_SLAM2/Examples/ROS && ./build_ros.sh


COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]


CMD ["bash", "-c", "rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml"]
