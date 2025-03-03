FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu22.04 as base
SHELL ["/bin/bash", "-c"]

ARG GCC_VERSION=12
ARG CLANG_VARION=17
ARG CMAKE_VERSION=3.28.3
ARG LIBTORCH_VERSION=2.3.0
ARG OPENCV_VERSION=4.11.0
ARG CUDA_ARCH=8.6

RUN apt-get update && apt-get install -y \
    gdb \
    git \
    python3 \
    python3-dev \
    tar \
    tree \
    unzip \
    wget \
    && apt-get clean


# Install ROS2 Humble
ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get install -y tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN sudo add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

################
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
RUN apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6



# # Install CMake
# RUN CMAKE_VERSION_MOC=$(echo ${CMAKE_VERSION} | rev | cut -c3- | rev) && wget "https://cmake.org/files/v${CMAKE_VERSION_MOC}/cmake-${CMAKE_VERSION}-linux-x86_64.sh" -q -O /tmp/cmake-install.sh
# RUN chmod u+x /tmp/cmake-install.sh
# RUN mkdir /opt/cmake-${CMAKE_VERSION}
# RUN /tmp/cmake-install.sh --skip-license --prefix=/opt/cmake-${CMAKE_VERSION}
# RUN rm /tmp/cmake-install.sh
# RUN ln -s /opt/cmake-${CMAKE_VERSION}/bin/cmake /usr/bin/cmake

#Install GCC
# RUN apt-get update && apt-get install -y software-properties-common && rm -rf /var/lib/apt/lists/*
# RUN add-apt-repository ppa:ubuntu-toolchain-r/test
# RUN apt-get update && apt-get install -y \
#     gcc-${GCC_VERSION} \
#     g++-${GCC_VERSION}
# RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-${GCC_VERSION} ${GCC_VERSION} --slave /usr/bin/g++ g++ /usr/bin/g++-${GCC_VERSION}

# # Install Clang
# RUN apt-get update && apt-get install -y lsb-release software-properties-common gnupg
# RUN wget https://apt.llvm.org/llvm.sh && chmod u+x llvm.sh
# RUN ./llvm.sh ${CLANG_VARION} all
# RUN ln -s /usr/bin/clang-${CLANG_VARION} /usr/bin/clang && ln -s /usr/bin/clang++-${CLANG_VARION} /usr/bin/clang++

# # Install Ninja
# RUN wget "https://github.com/ninja-build/ninja/releases/download/v${NINJA_VERSION}/ninja-linux.zip"
# RUN unzip ninja-linux.zip
# RUN mv ninja /usr/bin

# # Install LibTorch
# RUN wget "https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2Bcu121.zip"
# RUN unzip libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}+cu121.zip -d /usr/local/lib/
# RUN rm libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}+cu121.zip

# Install OpenCV
RUN apt-get update && apt-get install -y \
    build-essential \
    libgtk-3-dev \
    python3-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \ 
    libavformat-dev \ 
    libswscale-dev \
    libswresample-dev \
    libgstreamer1.0-dev \ 
    libgstreamer-plugins-base1.0-dev \
    libxvidcore-dev \
    x264 \
    libx264-dev \
    libfaac-dev \
    libmp3lame-dev \
    libtheora-dev \
    libfaac-dev \
    libmp3lame-dev \
    libvorbis-dev
RUN mkdir -p opencv-build/build
# Download OpenCV 4.11.0
RUN cd opencv-build && wget https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.zip \ 
    && unzip ${OPENCV_VERSION}.zip && rm ${OPENCV_VERSION}.zip \
    && git clone https://github.com/opencv/opencv_contrib.git
RUN cd opencv-build/build && cmake \
    -D WITH_TBB=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D BUILD_opencv_cudacodec=OFF \
    -D WITH_CUDA=ON \
    -D WITH_CUDNN=ON \
    -D CUDA_ARCH_BIN=${CUDA_ARCH} \
    -D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda  \
    -D CUDNN_LIBRARY=/usr/lib/x86_64-linux-gnu/libcudnn.so \
    -D WITH_GTK=ON \
    -D WITH_V4L=ON \
    -D WITH_QT=OFF \
    -D WITH_OPENGL=ON \
    -D WITH_GSTREAMER=ON \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D BUILD_EXAMPLES=OFF \
    -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules \
    ../opencv-${OPENCV_VERSION}
RUN cd opencv-build/build && make -j$(nproc) && make install
RUN rm -rf opencv-build 


################
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
RUN apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ENV DEBIAN_FRONTEND=

# Install cli tools via apt
RUN apt-get update \
    && apt-get install -y wget curl neovim python3-dev python3-numpy python3-pip\
    && rm -rf /var/lib/apt/lists/*

# Install rpi-ws281x-python package
RUN pip install open3d==0.18.0 numpy==1.26.4 ultralytics pytransform3d bayesian-optimization==1.5.1 cupy-cuda12x vcstool

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

# install rosdep
RUN apt-get update \
    && apt-get install -y python3-rosdep \
    && sudo rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Use Cyclone DDS as middleware
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Install MoveIt2
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-moveit

# Install UR
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ur

# Install joy package
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-joy

# Install ROS Control Cli
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros2controlcli \
    ros-${ROS_DISTRO}-rqt-controller-manager

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
