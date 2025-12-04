FROM osrf/ros:humble-desktop-full-jammy

# Set non-interactive frontend for apt-get
ENV DEBIAN_FRONTEND=noninteractive

# install necessary packages
RUN apt update && apt install -y --no-install-recommends \
    git \
    cmake \
    wget \
    lsb-release \
    gnupg \
    libqt5gui5 \
    ubuntu-gnome-desktop \
    g++ \
    python3 \
    freeglut3-dev \
    tmux \
    nano \
    gdb

# ========== MicroXRCE-DDS Agent ==========
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# ========== user ==========
ARG USERNAME=choreographer

# Create and use a user for the simulation
RUN useradd --system --user-group --create-home --shell /bin/bash --groups sudo $USERNAME
# Give sudo privileges without password
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
# Switch to the new user
USER $USERNAME
WORKDIR /home/$USERNAME

# Update tmux prefix (I USE C-a I FIND IT EASIER!)
RUN echo "set -g prefix C-a \nunbind C-b \nbind C-a send-prefix" >> ~/.tmux.conf

# Add ns3 to LD_LIBRARY_PATH so that shared libraries can be found
# Source ROS and workspace setup files in .bashrc
# Add ns-3 cmake folder to the PATH so that CMake can find libraries
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$USERNAME/ns3.42/lib" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/$USERNAME/sim_ws/install/setup.bash" >> ~/.bashrc && \
    echo "PATH=\$PATH:$HOME/ns3.42/lib/cmake" >> ~/.bashrc

# ========== ns-3 ==========
# Download ns-3 and unpack it
WORKDIR /home/$USERNAME
RUN wget https://www.nsnam.org/release/ns-allinone-3.42.tar.bz2 && \
tar -jxvf ns-allinone-3.42.tar.bz2

# install ns-3 in /home/$USERNAME/ns3
RUN cd ns-allinone-3.42/ns-3.42 &&\
./ns3 configure --prefix=/$HOME/ns3.42 --build-profile optimized --disable-asserts &&\
./ns3 install

# clean ns-3 archive
RUN rm ns-allinone-3.42.tar.bz2

# ========== gazebo ==========
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg &&\
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null &&\
    sudo apt update && sudo apt install -y \
    gz-harmonic

# ========== PX4 ==========
WORKDIR /home/$USERNAME
# Get the PX4 Autopilot sources (Latest version by default, uncomment for fixed version)
# RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN git clone https://github.com/PX4/PX4-Autopilot.git PX4-Autopilot-1.16 --recursive --branch release/1.16


WORKDIR /home/$USERNAME/PX4-Autopilot-1.16
# We need this commit to fix compatibility between Gazebo Harmonic and PX4 1.16
# RUN git fetch https://github.com/jmackay2/PX4-Autopilot.git fix_gazebo_harmonic:fix_gazebo_harmonic
# RUN git config --global user.name "$USERNAME" && \
#     git config --global user.email "$USERNAME@docker.com"
# RUN git cherry-pick bf4408b772f2bc398a5398dabd4bfa67a96ec1b5

# Build the PX4 Autopilot 
RUN ./Tools/setup/ubuntu.sh

WORKDIR /home/$USERNAME/PX4-Autopilot-1.16

# build the sitl executable
RUN make px4_sitl_default

# ========== QGroundControl ==========
# AppImages rely on FUSE, that is not easily usable in docker. Thus, it is needed to extract the content of the appimage in order to launch the executable directly
WORKDIR /home/$USERNAME
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage
RUN chmod +x QGroundControl-x86_64.AppImage
RUN ./QGroundControl-x86_64.AppImage --appimage-extract
RUN sudo ln -s ~/squashfs-root/AppRun /usr/local/bin/QGroundControl.AppImage

# Uncomment if you want to install QGroundControl inside the docker container, however it's hard to launch it from within the docker because docker prevents 
# the installation of 'fuse' that AppImages require. The cleaner approach would be build it from source, but even easier is to run QGC outside 
# of the docker container and open the container's network.
# USER root
# WORKDIR /home/$USERNAME
# RUN usermod -a -G dialout $USERNAME && \
#     apt-get remove modemmanager -y && \
#     apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y && \
#     apt install libfuse2 -y && \
#     apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y

# RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage

# RUN chmod +x QGroundControl.AppImage

# ========== DANCERS ==========
# Fake variable to rebuild only from here
ARG CACHE_BUST=1
WORKDIR /home/$USERNAME
# Get the source of the DANCERS components
RUN git clone -b v1.0 https://github.com/Chroma-CITI/DANCERS sim_ws --recursive

ENV ROS_WS=/home/$USERNAME/sim_ws

WORKDIR /home/$USERNAME/sim_ws

# Build all the ROS nodes (-i to run bash iteratively so that ~/.bashrc does not exit early | CMAKE -w flag to disable CMAKE warnings)
RUN /bin/bash -c -i "colcon build --cmake-args -DCMAKE_CXX_FLAGS='-w'"

