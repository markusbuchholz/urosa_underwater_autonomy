#Markus Buchholz, 2025
FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=rosuser
ENV USERNAME=${USERNAME}

RUN apt-get update && apt-get upgrade -y

RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    bash-completion \
    build-essential \
    git \
    glmark2 \
    gnupg \
    iputils-ping \
    lsb-release \
    mlocate \
    software-properties-common \
    sudo \
    wget \
    vim \
    nano \
    libxkbcommon-x11-0 \
    xvfb \
    libncurses5-dev \
    libncursesw5-dev \
    cmake \
    libzmq3-dev \
    libdw-dev \
    qtbase5-dev \
    libqt5svg5-dev \
  && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    gz-harmonic \
  && rm -rf /var/lib/apt/lists/*

# Install NVIDIA software
RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

WORKDIR /home/${USERNAME}/colcon_ws

# ENV DIST=humble
# RUN apt update \
#  && apt install -y --no-install-recommends \
#     ros-${DIST}-plotjuggler-ros \
#  && apt clean


# Install prerequisites for ArduPilot
RUN apt-get update && apt-get -y install \
    git \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    libtool \
    g++ \
    gcc \
    gawk \
    make \
    pkg-config \
    ccache \
    libncurses5-dev \
    gdb \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create a symbolic link for python -> python3
RUN ln -s /usr/bin/python3 /usr/bin/python

# Clone ArduPilot repository and checkout specific commit
WORKDIR /home/$USERNAME
RUN git clone https://github.com/ArduPilot/ardupilot.git
WORKDIR /home/$USERNAME/ardupilot
RUN git checkout aca9940534

# Initialize submodules
RUN git submodule update --init --recursive

# Install necessary Python dependencies, including pexpect
RUN pip3 install pymavlink MAVProxy dronekit pexpect PyPDF2

# Build ArduSub using waf-light
WORKDIR /home/$USERNAME/ardupilot
RUN modules/waf/waf-light configure --board sitl \
  && modules/waf/waf-light build --target bin/ardusub

# Update environment paths
RUN echo "export PATH=/home/$USERNAME/ardupilot/Tools/autotest:\$PATH" >> /root/.bashrc \
    && echo "export PATH=/home/$USERNAME/ardupilot/modules/waf:\$PATH" >> /root/.bashrc

RUN apt-get update && apt-get install -y \
ros-humble-tf-transformations \
&& rm -rf /var/lib/apt/lists/*

# Clean up to reduce image size
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Install FUSE
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    fuse \
  && rm -rf /var/lib/apt/lists/*

# Install BehaviorTree.CPP from the master branch with ZMQ support
RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git /opt/BehaviorTree.CPP \
    && cd /opt/BehaviorTree.CPP \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBTCPP_USE_ZMQ=true .. \
    && make \
    && make install

# Source ROS setup.bash and set environment variables for BehaviorTree.CPP
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:/usr/local" >> /root/.bashrc \
    && echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib" >> /root/.bashrc \
    && echo "export PKG_CONFIG_PATH=\$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig" >> /root/.bashrc \
    && echo "export PATH=\$PATH:/usr/local/bin" >> /root/.bashrc


# Set the working directory
WORKDIR /home/${USERNAME}/colcon_ws

# Install pip (needed for Python package installation)
RUN apt-get update && apt-get install -y python3-pip && rm -rf /var/lib/apt/lists/*

# Install OpenAI and python-dotenv Python packages system-wide (without the flag)
RUN pip3 install openai python-dotenv

RUN pip3 install --upgrade openai-whisper

RUN apt update \
  && apt install ffmpeg

RUN apt-get update && apt-get install -y python3-pip && rm -rf /var/lib/apt/lists/*

# Install OpenAI packages (and upgrade pip/setuptools/wheel if you want)
RUN pip3 install --upgrade pip setuptools wheel && \
    pip3 install openai python-dotenv openai-whisper

# Install other ML stack packages
RUN pip3 install --upgrade pip setuptools wheel && \
    pip3 install \
      numpy==1.24.3 \
      scipy==1.10.1 \
      opencv-python==4.11.0.86 \
      Pillow==11.2.1 \
      torch==2.7.0 torchvision==0.22.0 \
      transformers==4.52.4

RUN apt update && apt install -y ffmpeg

ENV DIST=humble

RUN apt update \
    && apt install -y --no-install-recommends \
    ros-${DIST}-plotjuggler-ros \
    && apt clean

# Entry point
CMD ["/bin/bash"]


