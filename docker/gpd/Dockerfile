ARG from=ubuntu:bionic
FROM ${from}
ARG PYTHON_VERSION=3.7
ARG ROS_DISTRO=noetic
ARG NJOBS=1
ARG GIT_USER="MatteoPerotto"
ARG GIT_EMAIL="perotto.matteo@gmail.com"


ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV TZ=Europe/Rome
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANGUAGE=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV WS_DIR=/home/user/workspace
ENV CATKIN_WS_DIR=${WS_DIR}/catkin_ws
ENV WS_INSTALL=${WS_DIR}/install
ENV WS_SOURCES=${WS_DIR}/sources
ENV WS_BIN=${WS_SOURCES}/bin

# Change default shell to bash
SHELL ["/bin/bash", "-c"]

# Install packages
# ================
RUN apt-get update &&\
    apt-get install -y --no-install-recommends --fix-missing\
        sudo \
        gosu \
        software-properties-common \
        git \
        wget \
        curl \
        nano \
        gedit \
        unzip \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        mesa-common-dev \
        apt-transport-https \
        apt-utils \
        dbus-x11 \
        bash-completion \
        gdb \
        ssh \
        iputils-ping \
        gnupg \
        gnupg2 \
        xvfb \
        mercurial \
        freeglut3-dev \
        libflann-dev \
        &&\
    rm -rf /var/lib/apt/lists/*

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | apt-key add - && \
    apt-add-repository "deb https://apt.kitware.com/ubuntu/ `lsb_release -cs` main" &&\
    add-apt-repository ppa:ubuntu-toolchain-r/test &&\
    add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main" && \
    apt-get update &&\
    apt-get install -y --no-install-recommends --fix-missing\
        gcc-8 \
        g++-8 \
        cmake \
        cmake-curses-gui \
        build-essential \
        mc \
        lynx \
        libqhull* \
        pkg-config \
        libxmu-dev \
        libxi-dev \
        libboost-all-dev \
        libeigen3-dev \
        libusb-1.0-0-dev \
        libudev-dev \
        doxygen \
        graphviz \
        libgtest-dev \
        libpcap-dev \
        libgtk2.0-dev \
        libavcodec-dev \
        libavformat-dev \
        libjpeg.dev \
        libtiff4.dev \
        libswscale-dev \
        libjasper-dev \
        &&\
    rm -rf /var/lib/apt/lists/*
    
# Setup locales and timezone
RUN apt-get update &&\
    apt-get install -y --no-install-recommends locales locales-all tzdata &&\
    rm -rf /var/lib/apt/lists/* &&\
    echo "${TZ}" | tee /etc/timezone &&\
    dpkg-reconfigure --frontend noninteractive tzdata
    
# Install GitHub cli
RUN curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg && \
    chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null && \
    apt update && \
    apt install --no-install-recommends -y -qq gh && \
    rm -rf /var/lib/apt/lists/*
    
# Install python3
# ===============
RUN add-apt-repository ppa:deadsnakes/ppa &&\
    apt-get update &&\
    apt-get install -y --no-install-recommends \
        python3 \
        python3-dev \
        libpython3-dev \
        python3-pip \
        python3-setuptools \
        python3-wheel \
        python3-pygments \
        python3-numpy \
        &&\
    rm -rf /var/lib/apt/lists/*


# Install ROS Desktop Full
# ========================
RUN apt-key adv --keyserver keyserver.ubuntu.com \
                --recv-keys F42ED6FBAB17C654 &&\
    echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" \
        > /etc/apt/sources.list.d/ros-latest.list &&\
    apt-get update &&\
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop-full \
	ros-${ROS_DISTRO}-jsk-tools \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-vcstools \
        python3-wstool \
        python3-catkin-tools \
        python3-catkin-pkg-modules \
        python3-rospkg-modules \
        python3-empy \
        &&\
    rm -rf /var/lib/apt/lists/* &&\
    rosdep init &&\
    rosdep update 

# Install VTK
# ============
RUN apt-get update &&\
    apt-get install -y\
    libvtk7* &&\
    rm -rf /var/lib/apt/lists/*

# Create user with passwordless sudo
RUN useradd -l -G sudo -md /home/user -s /bin/bash -p user user && \
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers
USER user

# Install robotology superbuild 
WORKDIR /home/user 

RUN git config --global user.name ${GIT_USER} && \
    git config --global user.email ${GIT_EMAIL} && \
    git clone https://github.com/robotology/robotology-superbuild  && \
    cd robotology-superbuild && git checkout v2022.09.1 && \
    sudo apt-get update && \
    sudo bash scripts/install_apt_dependencies.sh && \
    sudo rm -rf /var/lib/apt/lists/*
	
RUN cd /home/user/robotology-superbuild && \
    mkdir build && cd build && \
    cmake -DROBOTOLOGY_ENABLE_CORE=ON -DROBOTOLOGY_ENABLE_ICUB_HEAD=ON -DROBOTOLOGY_USES_GAZEBO=OFF -DROBOTOLOGY_USES_PYTHON=ON -DYARP_COMPILE_BINDINGS=ON ../ && \
    make -j$NJOBS
    
# Install Eigen
# =============
RUN mkdir -p ${WS_SOURCES} &&\
    cd ${WS_SOURCES} &&\
    git clone -b 3.2 https://gitlab.com/libeigen/eigen.git eigen &&\
    cd eigen &&\
    mkdir build &&\
    cd build &&\
    cmake  -D CMAKE_INSTALL_PREFIX=${WS_INSTALL} .. &&\
    make -j8 &&\
    make install 

# Install PCL
# ===========
RUN cd ${WS_SOURCES}\
    && wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.0.zip \
    && unzip pcl-1.9.0.zip \
    && cd pcl-pcl-1.9.0 \
    && mkdir build \
    && cd build \
    && cmake -D CMAKE_BUILD_TYPE=Release -D BUILD_GPU=OFF -D BUILD_apps=ON -D BUILD_examples=ON  -DCMAKE_INSTALL_PREFIX=${WS_INSTALL} .. \
    && make -j8 \
    && make install 

# Install Opencv
# ==============
RUN cd ${WS_SOURCES}\
    && wget https://github.com/opencv/opencv/archive/3.4.3.zip \
    && unzip 3.4.3.zip \
    && cd opencv-3.4.3 \
    && mkdir build \
    && cd build \
    && cmake -D WITH_OPENMP=ON -D ENABLE_PRECOMPILED_HEADERS=OFF  -DCMAKE_INSTALL_PREFIX=${WS_INSTALL} ..\
    && make -j 8 \
    && make install

# Install CAFFE
# =============
# RUN apt-get update &&\
#     apt-get install -y \
#     libprotobuf-dev \
#     libleveldb-dev \
#     libsnappy-dev \
#     libopencv-dev \
#     libhdf5-serial-dev \
#     protobuf-compiler \
#     libboost-all-dev \
#     libatlas-base-dev \
#     liblmdb-dev \
#     libturbojpeg \
#     libturbojpeg0-dev \
#     libgoogle-glog-dev &&\
#     rm -rf /var/lib/apt/lists/* &&\
#     cd ${WS_SOURCES}\
#     && git clone https://github.com/fbottarel/caffe.git &&\
#     cd ${WS_SOURCES}/caffe \
#     && mkdir build \
#     && cd build \
#     && cmake -D BUILD_python=OFF \
#         -D BUILD_python_layer=OFF \
#         -D BLAS=Atlas \
#         -D CUDA_rt_LIBRARY=/usr/local/cuda/lib64/libcudart.so \
#         -D CMAKE_INSTALL_PREFIX=${WS_INSTALL} \
#         ../ \
#     && make -j 8 \
#     && make install 

# Install gpd
# ===========
RUN cd ${WS_SOURCES}\
    && git clone https://github.com/atenpas/gpd gpd \
    && cd gpd \
    && mkdir build \
    && cd build \
    && cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=${WS_INSTALL}\
        -D USE_CAFFE=OFF \
        .. \
    && make -j8  \
    && make install 

# Install hyperpcr (and dependencies)
# ===========
RUN cd ${WS_SOURCES}\
    && git clone https://github.com/andrearosasco/hyperpcr.git \
    && pip3 install --upgrade pip \
    && pip3 install --ignore-installed open3d hyperpcr 

# Create catkin_ws with dependencies/ros packages for python3
RUN mkdir -p ${CATKIN_WS_DIR}/src && \
    cd ${CATKIN_WS_DIR}/src &&\
    git clone -b fc_grasp_demo https://github.com/MatteoPerotto/grasping-benchmarks-panda.git &&\
    pip3 install grasping-benchmarks-panda/ &&\
    cd ${CATKIN_WS_DIR} &&\
    catkin config --extend /opt/ros/${ROS_DISTRO} -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release -DGPD_ROS=ON -DPCL_DIR=${WS_INSTALL}/share/pcl-1.9 &&\
    catkin build  -j8 
    
# Source things 
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
	echo 'source ${CATKIN_WS_DIR}/devel/setup.bash' >> ~/.bashrc && \
	echo 'export LD_LIBRARY_PATH=${WS_INSTALL}/lib:${LD_LIBRARY_PATH}' >> ~/.bashrc && \
	echo "source /home/user/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh" >> ~/.bashrc


# Image initialization
# ====================
#ENV USERNAME=docker

# Include a custom bashrc
#COPY bashrc /usr/etc/skel/bashrc-dev

#COPY setup.sh /usr/sbin/setup.sh
#COPY entrypoint.sh /usr/sbin/entrypoint.sh
#RUN chmod 755 /usr/sbin/setup.sh
#RUN chmod 755 /usr/sbin/entrypoint.sh
#ENTRYPOINT ["/usr/sbin/entrypoint.sh"]
CMD ["bash"]
