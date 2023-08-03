FROM dustynv/ros:humble-pytorch-l4t-r35.3.1

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash

ENV ROS_PYTHON_VERSION=3.8

ENV ROS_ROOT=/opt/ros/humble
ENV ROS_DISTRO=humble
ENV ROS_PKG=desktop

WORKDIR ${ROS_ROOT}

RUN mkdir src

# download ROS sources
# https://answers.ros.org/question/325245/minimal-ros2-installation/?answer=325249#post-id-325249
RUN rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ${ROS_PKG} \
	launch_xml \
	launch_yaml \
	launch_testing \
	launch_testing_ament_cmake \
	demo_nodes_cpp \
	demo_nodes_py \
	example_interfaces \
	camera_calibration_parsers \
	camera_info_manager \
	cv_bridge \
	v4l2_camera \
	vision_opencv \
	vision_msgs \
	image_geometry \
	image_pipeline \
	image_transport \
	compressed_image_transport \
	compressed_depth_image_transport \
> ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall

RUN cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall
RUN vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall
    
# https://github.com/dusty-nv/jetson-containers/issues/181
RUN rm -r ${ROS_ROOT}/src/ament_cmake
RUN git -C ${ROS_ROOT}/src/ clone https://github.com/ament/ament_cmake -b ${ROS_DISTRO}

# skip installation of some conflicting packages
ARG SKIP_KEYS="rti-connext-dds-6.0.1 ignition-cmake2 ignition-math6 libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"

# patches for building Humble on 18.04

	# the default gcc-7 is too old to build humble
#RUN apt-get install -y --no-install-recommends gcc-8 g++-8
#RUN export CC="/usr/bin/gcc-8"
#RUN export CXX="/usr/bin/g++-8"
#RUN echo "CC=$CC CXX=$CXX"

RUN apt update

# upgrade pybind11
RUN apt-get purge -y pybind11-dev
RUN pip3 install --upgrade --no-cache-dir pybind11-global
   
# https://github.com/dusty-nv/jetson-containers/issues/160#issuecomment-1429572145
RUN git -C /tmp clone -b yaml-cpp-0.6.0 https://github.com/jbeder/yaml-cpp.git
RUN cmake -S /tmp/yaml-cpp -B /tmp/yaml-cpp/BUILD -DBUILD_SHARED_LIBS=ON
RUN cmake --build /tmp/yaml-cpp/BUILD --parallel $(nproc --ignore=1)
RUN cmake --install /tmp/yaml-cpp/BUILD
RUN rm -rf /tmp/yaml-cpp

    
RUN echo "--skip-keys ${SKIP_KEYS}"
    
# install dependencies using rosdep
# RUN rosdep init
RUN rosdep update
RUN rosdep install -y \
	--ignore-src \
	--from-paths src \
	--rosdistro ${ROS_DISTRO} \
	--skip-keys "${SKIP_KEYS}"

# build it all - for verbose, see https://answers.ros.org/question/363112/how-to-see-compiler-invocation-in-colcon-build
RUN colcon build \
	--merge-install \
	--cmake-args -DCMAKE_BUILD_TYPE=Release 
    
# remove build files
RUN rm -rf ${ROS_ROOT}/src
RUN rm -rf ${ROS_ROOT}/logs
RUN rm -rf ${ROS_ROOT}/build
RUN rm ${ROS_ROOT}/*.rosinstall

#WORKDIR /

COPY ./scripts/ros_entrypoint.sh ros_entrypoint.sh
# RUN echo 'source /ros_entrypoint.sh' >> /root/.bashrc
# sudo docker run -it --rm --net=host --runtime nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3

# install ros_deep_learning package (PyTorch-based container only)
#COPY ./scripts/ros_deep_learning.sh ros_deep_learning.sh
#RUN ./ros_deep_learning.sh
