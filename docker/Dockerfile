ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}

#Install extra ROS packages for Isaac Perceptor
RUN apt update && apt-get upgrade -y

#Install depthai ros package for the OAK-D ROS driver
RUN wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
RUN python3 -m pip install depthai
RUN apt install -y ros-humble-depthai-ros

RUN apt install -y \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-cv-bridge \
  ros-humble-foxglove-msgs

RUN apt-get update && apt-get install -y --no-install-recommends \
  pkg-config \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-plugins-rtp \
  gstreamer1.0-x \
  gstreamer1.0-gl \
  python3-gst-1.0 \
  libgstrtspserver-1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  gstreamer1.0-tools \
  gstreamer1.0-alsa \
  gstreamer1.0-gtk3 \
  gstreamer1.0-qt5 \
  gstreamer1.0-pulseaudio \
  v4l-utils \
  && rm -rf /var/lib/apt/lists/*