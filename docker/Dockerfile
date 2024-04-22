# 使用官方ROS Noetic作为基础镜像
FROM ros:noetic-ros-base-focal

# 设置非交互式安装，避免安装过程中的用户输入提示
ENV DEBIAN_FRONTEND noninteractive

# 安装ISSPA所需的依赖
RUN apt update && apt install -y \
    libuvc-dev \
    libgoogle-glog-dev \
    ros-noetic-costmap-2d \
    ros-noetic-nav-core \
    libceres-dev \
    git \
    ros-noetic-desktop \
    && rm -rf /var/lib/apt/lists/*


# 克隆ISSPA源代码
WORKDIR /home/root
RUN git clone https://github.com/chenhengwei1999/ISSPA.git

# 编译ISSPA工作空间
WORKDIR /home/root/ISSPA

RUN apt-get update && apt-get install -y \
    ros-noetic-image-geometry \
    ros-noetic-camera-info-manager \
    ros-noetic-desktop-full
#RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; rosdep install --from-paths src --ignore-src -r -y'
RUN apt-get install -y \
    ros-noetic-imu-filter-madgwick \
    ros-noetic-robot-localization \
    pip
RUN pip install pyserial
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make VERBOSE=1'

# 设置环境变量
RUN echo "source /home/root/ISSPA/devel/setup.bash" >> ~/.bashrc
RUN ln -s /usr/bin/python3 /usr/bin/python
# SSH
RUN apt-get install -y ros-noetic-rosbridge-server
# ??????????OpenSSH???
RUN apt-get install -y openssh-server && \
    rm -rf /var/lib/apt/lists/*

# ??SSH???????
RUN mkdir /var/run/sshd

# ???????????????????????
RUN useradd -m -s /bin/bash username && \
    echo "username:userpassword" | chpasswd

# ??SSH??????????????????????
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# ??22??
EXPOSE 22

# ??SSH??
# RUN service ssh start
CMD ["service", "ssh", "start"]

# 设置容器启动时执行的命令
CMD ["bash"]
