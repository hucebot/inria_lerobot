FROM nvidia/cuda:12.4.1-base-ubuntu22.04

###### Set environment variable
ENV DISPLAY=:0
ENV ROS_DISTRO=noetic
ENV LIBGL_ALWAYS_INDIRECT=0

###### Configure environment variables
ARG PYTHON_VERSION=3.10
ENV DEBIAN_FRONTEND=noninteractive
ENV MUJOCO_GL="egl"
ENV PATH="/opt/venv/bin:$PATH"

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="UTC"

###### Install basic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    libglib2.0-0 libgl1-mesa-glx libegl1-mesa ffmpeg \
    speech-dispatcher libgeos-dev \
    python${PYTHON_VERSION}-dev \
    python-is-python3 \
    python3-pip \
    terminator

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    libglib2.0-0 libgl1-mesa-glx libegl1-mesa ffmpeg \
    speech-dispatcher libgeos-dev

RUN apt-get install -y --reinstall \
    libqt5core5a libqt5gui5 libqt5widgets5 \
    libxcb-xinerama0 libxkbcommon-x11-0 \
    libqt5x11extras5

RUN pip install pyserial \
    feetech-servo-sdk \
    pynput \
    numpy

##### 
RUN pip install torch torchvision
RUN pip install opencv-python
RUN pip install torchdiffeq pandas matplotlib
RUN pip install streamdeck pillow 

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