FROM osrf/ros:noetic-desktop-full
LABEL authors="RegisLab"

RUN apt-get update && apt-get install -y \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libgl1-mesa-dev

# Install git
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y git
RUN apt-get install -y wget

# Clone and build PX4
RUN git clone https://github.com/PX4/PX4-Autopilot.git
WORKDIR PX4-Autopilot
RUN git checkout v1.13.3
RUN /bin/bash Tools/setup/ubuntu.sh
RUN DONT_RUN=1 make px4_sitl_default gazebo -j$(nproc)

COPY scripts/install_ros_tools_for_uav.sh /install_tmp.sh
RUN /install_tmp.sh
RUN rm /install_tmp.sh

#install opencv
RUN sudo apt update
RUN sudo apt install python3-pip -y
RUN sudo apt install libopencv-dev -y
RUN pip install opencv-python==4.8.0.76
RUN pip install opencv-contrib-python==4.8.0.76


# install and run ssh server
RUN apt-get update && \
    apt-get install -y openssh-server && \
    mkdir /var/run/sshd
#
RUN echo 'root:root' | chpasswd
# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin no/' /etc/ssh/sshd_config
RUN echo 'AllowUsers myuser' >> /etc/ssh/sshd_config

# Install sudo package
RUN apt-get update && apt-get install -y sudo
RUN apt install python3.8-venv -y
RUN apt install python3-pcl -y
RUN apt-get install ros-noetic-tf2-sensor-msgs -y
RUN apt-get install python3-tk -y

# Setup user environment 
RUN useradd -ms /bin/bash docker && echo "docker:docker" | chpasswd && adduser docker sudo
RUN usermod -aG sudo docker
RUN echo 'docker ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
EXPOSE 22
USER docker
RUN pip install matplotlib==3.5.1
RUN pip install packaging==23.2

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
RUN mkdir -p /home/docker/.gazebo/paging/heightmap

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

CMD /bin/bash