FROM --platform=x86_64 nvidia/cuda:12.1.0-cudnn8-devel-ubuntu20.04
ARG CUDNN_ARCH="linux-x86_64"

##### Set the default shell to bash
SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

##### Set the timezone
ENV TZ="Europe/Paris"
ARG DEBIAN_FRONTEND=noninteractive
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

##### Install Python 3.10 and set it as the default
RUN apt-get update && apt-get install -y software-properties-common curl && \
    add-apt-repository ppa:deadsnakes/ppa -y && \
    apt-get update && \
    apt-get install -y python3.10 python3.10-dev && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 && \
    update-alternatives --install /usr/bin/python python /usr/bin/python3.10 1 && \
    curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10 && \
    pip install --upgrade pip setuptools wheel

WORKDIR /deps

##### Install required packages
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    git ninja-build cmake build-essential \
    libopenblas-dev clang unzip wget software-properties-common bash-completion \
    libx11-dev python3-tk neovim xorg-dev autoconf libtool ffmpeg portaudio19-dev \
    python3-opencv python3-pyaudio terminator libxext-dev x11proto-gl-dev curl \
    zlib1g-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev libatlas-base-dev \
    make lsb-release tree sudo gnupg-agent libssl-dev apt-transport-https \
    gnupg2 usbutils mesa-utils mesa-va-drivers vainfo locales iputils-ping \
    libv4l-dev v4l-utils libnuma-dev libnuma1 libgles2-mesa libeigen3-dev \
    ca-certificates && \
    rm -rf /var/lib/apt/lists/*

##### Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt install curl -y
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add
RUN apt update
RUN apt install ros-noetic-desktop-full -y
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

##### Fix ROS dependencies
RUN pip uninstall -y netifaces && pip install netifaces
RUN pip install psutil
RUN python3.10 -m pip install --upgrade --force-reinstall --ignore-installed pycryptodomex

##### Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    libglib2.0-0 libgl1-mesa-glx libegl1-mesa ffmpeg \
    libgeos-dev \
    libqt5core5a libqt5gui5 libqt5widgets5 \
    libxcb-xinerama0 libxkbcommon-x11-0 libqt5x11extras5 \
    speech-dispatcher

###### Install packages for libusb
RUN apt-get install -yy python3-setuptools \
    && apt-get install -yy python3-pip \
    && apt-get install -yy python3-dev \
    && apt-get install -yy libusb-1.0-0-dev \
    && apt-get install -yy libudev-dev \
    && apt-get install -yy libhidapi-libusb0 \
    && apt-get install -yy libjpeg-dev \
    && apt-get install -yy zlib1g-dev \
    && apt-get install -yy libopenjp2-7 \
    && apt-get install -yy libtiff5

##### Install python packages
RUN python3.10 -m pip install --upgrade pip setuptools wheel && \
    python3.10 -m pip install --no-cache-dir -I \
    pynput numpy lerobot && \
    python3.10 -m pip install --no-cache-dir \
    torch torchvision torchdiffeq opencv-python pandas matplotlib pyserial feetech-servo-sdk
