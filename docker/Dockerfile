FROM nvidia/cuda:12.4.1-base-ubuntu20.04

###### Set environment variables
ENV DISPLAY=:0
ENV ROS_DISTRO=noetic
ENV LIBGL_ALWAYS_INDIRECT=0
ENV DEBIAN_FRONTEND=noninteractive
ENV MUJOCO_GL="egl"
ENV TZ="UTC"
ENV PATH="/opt/venv/bin:$PATH"

ENV PYTHONPATH=/usr/local/lib/python3.10/dist-packages:$PYTHONPATH

###### Install basic utilities and Python 3.10
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common curl gnupg2 lsb-release tzdata ca-certificates && \
    add-apt-repository ppa:deadsnakes/ppa -y && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    python3.10 python3.10-dev python3-pip python3-venv python-is-python3 && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 && \
    update-alternatives --install /usr/bin/python python /usr/bin/python3.10 1 && \
    curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10 && \
    pip install --upgrade pip setuptools wheel

###### Install basic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    libglib2.0-0 libgl1-mesa-glx libegl1-mesa ffmpeg \
    libgeos-dev \
    libqt5core5a libqt5gui5 libqt5widgets5 \
    libxcb-xinerama0 libxkbcommon-x11-0 libqt5x11extras5 \
    terminator speech-dispatcher

###### Install Python packages
RUN pip install \
    pynput numpy lerobot \
    torch torchvision \
    opencv-python \
    torchdiffeq pandas matplotlib \
    pycryptodome

###### Clean system Cryptodome
RUN rm -rf /usr/lib/python3/dist-packages/Cryptodome

###### Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-noetic-ros-base \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool

###### Install ROS basic packages
RUN apt-get install -y --no-install-recommends \
    ros-noetic-geometry-msgs \
    ros-noetic-std-msgs

RUN pip install pyserial \
    feetech-servo-sdk \
    pynput \
    numpy

###### Fix ROS executable shebangs
RUN find /opt/ros/noetic/bin/ -type f -exec sed -i '1 s|^#!.*|#!/usr/bin/env python3|' {} \;

###### Source ROS environment by default
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
