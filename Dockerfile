ARG BASE_IMAGE=nvidia/cuda:11.8.0-base-ubuntu22.04
FROM ${BASE_IMAGE} AS downloader

# Determine Webots version to be used and set default argument
ARG WEBOTS_VERSION=R2023b
ARG WEBOTS_PACKAGE_PREFIX=

# Disable dpkg/gdebi interactive dialogs
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --yes wget bzip2 && rm -rf /var/lib/apt/lists/ && \
 wget https://github.com/cyberbotics/webots/releases/download/$WEBOTS_VERSION/webots-$WEBOTS_VERSION-x86-64$WEBOTS_PACKAGE_PREFIX.tar.bz2 && \
 tar xjf webots-*.tar.bz2 && rm webots-*.tar.bz2

FROM ${BASE_IMAGE}

# Disable dpkg/gdebi interactive dialogs
ENV DEBIAN_FRONTEND=noninteractive

# Install Webots runtime dependencies
RUN apt-get update && apt-get install --yes wget xvfb locales && rm -rf /var/lib/apt/lists/ && \
  wget https://raw.githubusercontent.com/cyberbotics/webots/master/scripts/install/linux_runtime_dependencies.sh && \
  chmod +x linux_runtime_dependencies.sh && ./linux_runtime_dependencies.sh && rm ./linux_runtime_dependencies.sh && rm -rf /var/lib/apt/lists/

# Install Webots
WORKDIR /usr/local
COPY --from=downloader /webots /usr/local/webots/
ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV WEBOTS_HOME /usr/local/webots
ENV PATH /usr/local/webots:${PATH}

# Enable OpenGL capabilities
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility

# Set a user name to fix a warning
ENV USER root

# Set the locales
RUN locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

# Install ROS 2 (Humble)
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros2-control\
    ros-humble-ros2-controllers\
    ros-humble-webots-ros2 \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Finally open a bash command to let the user interact
CMD ["/bin/bash"]
