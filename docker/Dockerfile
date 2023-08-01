FROM nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash

ENV ROS_PYTHON_VERSION=3.8

ENV ROS_ROOT=/opt/ros/iron

WORKDIR /

RUN apt-get update
RUN apt-get purge -y '*opencv*' || echo "existing OpenCV installation not found"
RUN apt-get install -y --no-install-recommends \
          cmake \
		  nano \
		  mesa-utils \
		  lsb-release \
		  gstreamer1.0-tools \
		  gstreamer1.0-libav \
		  gstreamer1.0-rtsp \
		  gstreamer1.0-plugins-good \
		  gstreamer1.0-plugins-bad \
		  gstreamer1.0-plugins-ugly \
		  libgstreamer-plugins-base1.0-dev \
		  libgstreamer-plugins-good1.0-dev \
		  libgstreamer-plugins-bad1.0-dev

RUN apt-get install -y --no-install-recommends \
		  gstreamer1.0-plugins-rtp

RUN rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN mkdir -p /usr/local/include/gstreamer-1.0/gst && \
    cp -r /usr/include/gstreamer-1.0/gst/webrtc /usr/local/include/gstreamer-1.0/gst && \
    ls -ll /usr/local/include/ && \
    ls -ll /usr/local/include/gstreamer-1.0/gst/webrtc

RUN apt-get update 
RUN apt-get install -y --no-install-recommends git cmake libpython3-dev python3-numpy
RUN git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
RUN mkdir -p jetson-inference/build  && \
    cd jetson-inference/build && \
    cmake -DENABLE_NVMM=off ../ && \
    make -j$(nproc) && \
    make install && \
    ldconfig

RUN locale  
RUN apt update && sudo apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install -y --no-install-recommends software-properties-common
RUN add-apt-repository universe

RUN apt update && sudo apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && sudo apt install -y \
  python3-pip 

RUN apt-get install -y --no-install-recommends \
		build-essential \
		cmake \
		git \
		libbullet-dev \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-flake8 \
		python3-pip \
		python3-numpy \
		python3-pytest-cov \
		python3-rosdep \
		python3-setuptools \
		python3-vcstool \
		python3-rosinstall-generator \
		libasio-dev \
		libtinyxml2-dev \
		libcunit1-dev

# install some pip packages needed for testing
RUN pip3 install --upgrade --no-cache-dir \
		argcomplete \
		flake8-blind-except \
		flake8-builtins \
		flake8-class-newline \
		flake8-comprehensions \
		flake8-deprecated \
		flake8-docstrings \
		flake8-import-order \
		flake8-quotes \
		pytest-repeat \
		pytest-rerunfailures \
		pytest
  
RUN python3 -m pip install --upgrade pip
RUN pip3 install --no-cache-dir scikit-build
RUN pip3 install --upgrade --no-cache-dir --verbose cmake
RUN cmake --version
RUN which cmake

RUN apt purge -y python3.9 libpython3.9* || echo "python3.9 not found, skipping removal"
RUN ls -ll /usr/bin/python*

RUN apt update && sudo apt install -y ros-dev-tools

RUN mkdir -p /opt/ros/iron/src 

WORKDIR /opt/ros/iron

RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos src

RUN apt upgrade -y

RUN rosdep init
RUN rosdep update

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1
RUN PYTHON3_VERSION=`python3 -c 'import sys; version=sys.version_info[:3]; print("{0}.{1}".format(*version))'`

RUN rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"

RUN colcon build --symlink-install

WORKDIR /opt/ros/iron/src
RUN git clone https://github.com/ros-perception/vision_msgs.git

WORKDIR /opt/ros/iron
RUN colcon build --packages-select vision_msgs --symlink-install

RUN apt install -y vim 

WORKDIR /

COPY ./scripts/ros_entrypoint.sh ros_entrypoint.sh
RUN echo 'source /ros_entrypoint.sh' >> /root/.bashrc
# sudo docker run -it --rm --net=host --runtime nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3

# install ros_deep_learning package (PyTorch-based container only)
COPY ./scripts/ros_deep_learning.sh ros_deep_learning.sh
RUN ./ros_deep_learning.sh
