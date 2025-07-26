FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# ✅ ROSリポジトリの鍵とsource.list追加（Noetic用）
RUN apt-get update && apt-get install -y curl gnupg lsb-release && \
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xF42ED6FBAB17C654' | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list

# ✅ パッケージインストール
RUN apt-get update && apt-get install -y \
    git \
    wget \
    nano \
    xauth \
    vim-common \
    python3-pip \
    python3-catkin-tools \
    kbd \
    ros-noetic-joy \
    ros-noetic-moveit-commander \
    ros-noetic-moveit-ros-visualization \
    ros-noetic-moveit-ros-move-group \
    ros-noetic-moveit-simple-controller-manager \
    ros-noetic-moveit-fake-controller-manager \
    ros-noetic-moveit-kinematics \
    ros-noetic-moveit-planners-ompl \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-gripper-action-controller \
    ros-noetic-dynamixel-sdk \
    ros-noetic-roboticsgroup-upatras-gazebo-plugins \
    && rm -rf /var/lib/apt/lists/*

# ✅ Python ライブラリ（keyboardなど）
RUN pip3 install keyboard

# ✅ ワークスペース作成とコード取得
RUN mkdir -p /root/ws/src
WORKDIR /root/ws/src

RUN git clone https://github.com/yasuohayashibara/crane-x7-teaching-playback.git
RUN git clone -b master https://github.com/rt-net/crane_x7_ros.git
RUN git clone -b master https://github.com/rt-net/crane_x7_description.git

# ✅ rosdep
WORKDIR /root/ws
RUN rosdep update --include-eol-distros && \
    rosdep install --from-paths src --ignore-src -r -y

# ✅ ビルド
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# ✅ 起動時に環境を source
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws/devel/setup.bash" >> /root/.bashrc

# ✅ GUI対応のための環境変数
ENV QT_X11_NO_MITSHM=1

# ✅ エントリポイント追加
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

