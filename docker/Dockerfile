ARG ROS_DISTRO=jazzy

##########################################
# Base Image for Inspection Tool Control #
##########################################
FROM osrf/ros:${ROS_DISTRO}-desktop-full as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install cli tools via apt
RUN apt-get update \
    && apt-get install -y wget curl neovim python3-dev python3-numpy python3-pip\
    && rm -rf /var/lib/apt/lists/*

# Install rpi-ws281x-python package
RUN pip install open3d==0.18.0 numpy==1.26.4 ultralytics pytransform3d bayesian-optimization==1.5.1 cupy-cuda12x

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y \
    build-essential \
    cmake \
    gcc \
    g++ \
    libavcodec-dev libavformat-dev libswscale-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
    libgtk-3-dev \
    libpng-dev libjpeg-dev libopenexr-dev libtiff-dev libwebp-dev \
    git-all \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# OpenCV with CUDA Support

ARG OPENCV_VERSION="4.10.0"

# Install build dependencies
RUN apt-get clean && \
    apt-get update && \
    apt-get install -y --no-install-recommends --fix-missing \
    build-essential binutils \
    ca-certificates cmake cmake-qt-gui curl \
    dbus-x11 \
    ffmpeg \
    gdb gcc g++ gfortran git \
    tar \
    lsb-release \
    procps \
    manpages-dev \
    unzip \
    zip \
    wget \
    xauth \
    swig \
    python3-pip python3-dev python3-numpy python3-distutils \
    python3-setuptools python3-pyqt5 python3-opencv \
    libboost-python-dev libboost-thread-dev libatlas-base-dev libavcodec-dev \
    libavformat-dev libavutil-dev libcanberra-gtk3-module libeigen3-dev \
    libglew-dev libgl1-mesa-dev libgl1-mesa-glx libglib2.0-0 libgtk2.0-dev \
    libgtk-3-dev libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev liblapack-dev \
    liblapacke-dev libopenblas-dev libopencv-dev libpng-dev libpostproc-dev \
    libpq-dev libsm6 libswscale-dev libtbb-dev libtbb2 libtesseract-dev \
    libtiff-dev libtiff5-dev libv4l-dev libx11-dev libxext6 libxine2-dev \
    libxrender-dev libxvidcore-dev libx264-dev libgtkglext1 libgtkglext1-dev \
    libvtk9-dev libdc1394-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev libopenexr-dev \
    openexr \
    pkg-config \
    qv4l2 \
    v4l-utils \
    zlib1g-dev \
    locales \
    && locale-gen en_US.UTF-8 \
    && LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN apt-get update && apt-get install -y wget gnupg2 lsb-release
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
RUN dpkg -i cuda-keyring_1.1-1_all.deb
RUN apt-get update && apt-get install -y cuda-toolkit-12-5

WORKDIR $HOME
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip \
    && wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip \
    && unzip opencv.zip \
    && unzip opencv_contrib.zip \
    && mv opencv-${OPENCV_VERSION} opencv \
    && mv opencv_contrib-${OPENCV_VERSION} opencv_contrib

RUN mkdir -p $HOME/opencv/build
WORKDIR $HOME/opencv/build

# RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
#     -D CMAKE_INSTALL_PREFIX=/usr/local \
#     -D WITH_CUDA=ON \
#     -D CUDA_ARCH_BIN=8.6 \
#     -D INSTALL_PYTHON_EXAMPLES=ON \
#     -D INSTALL_C_EXAMPLES=ON \
#     -D OPENCV_ENABLE_NONFREE=ON \
#     -D OPENCV_GENERATE_PKGCONFIG=ON \
#     -D OPENCV_EXTRA_MODULES_PATH=$HOME/opencv_contrib/modules \
#     -D PYTHON_EXECUTABLE=/usr/local/bin/python \
#     -D BUILD_EXAMPLES=ON .. \
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local \ 
    -D WITH_CUDA=ON \
    -D WITH_CUDNN=ON \
    -D WITH_CUBLAS=ON \
    -D WITH_TBB=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D HAVE_opencv_cudev=ON \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D CUDA_ARCH_BIN=8.6 \
    -D OPENCV_EXTRA_MODULES_PATH=$HOME/opencv_contrib/modules \
    -D BUILD_EXAMPLES=OFF \
    -D HAVE_opencv_python3=ON \
    -D ENABLE_FAST_MATH=1 \
    -D cuda_toolkit_root_dir=/usr/local/cuda-12.6  \
    -D CUDNN_INCLUDE_DIR=/usr/include/  \
    -D CUDNN_LIBRARY=/usr/lib/x86_64-linux-gnu/libcudnn.so.8 .. \
    && make -j$(nproc) && make install && ldconfig

RUN sudo ln -s /usr/local/lib/python3.10/site-packages/cv2 /usr/local/lib/python3.10/dist-packages/cv2



# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

ARG DEBIAN_FRONTEND=dialog

# Create base Colcon workspace with external dependencies
RUN mkdir -p /base_ws/src
WORKDIR /base_ws/src
COPY base_dependencies.repos .
RUN vcs import < base_dependencies.repos

WORKDIR /base_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install --executor sequential

RUN apt-get update \
    && apt-get install -y ros-humble-ur

#############################################
# Overlay Image for Inspection Tool Control #
#############################################
FROM base AS overlay

# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws/src
COPY overlay_dependencies.repos .
RUN vcs import < overlay_dependencies.repos

WORKDIR /overlay_ws
RUN source /base_ws/install/setup.bash \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install

RUN mkdir -p /root/Inspection/Parts
# COPY Parts /root/Inspection/Parts

# Set environment variables
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
COPY ./bashrc /root/.bashrc
ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["bash"]

#############################################
# Devel Image for Inspection Tool Control #
#############################################
FROM base AS devel

# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws/src

WORKDIR /overlay_ws
RUN source /base_ws/install/setup.bash \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
    && colcon build --symlink-install

RUN mkdir -p /root/Inspection/Parts

# Set environment variables
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Set up the entrypoint
COPY ./docker/devel_entrypoint.sh /
COPY ./bashrc /root/.bashrc
ENTRYPOINT [ "/devel_entrypoint.sh" ]

CMD ["bash"]
