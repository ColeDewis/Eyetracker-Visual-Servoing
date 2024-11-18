FROM ghcr.io/ualberta-robotics/tobii-ros:master
ARG USER=user
ARG DEBIAN_FRONTEND=noninteractive

COPY requirements.txt requirements.txt
COPY requirements_3_10.txt requirements_3_10.txt
COPY packages.txt packages.txt

# realsense setup so we can use 405
RUN mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list

# install dependencies
RUN rm -rf /var/lib/apt/lists/* && apt-get clean
RUN apt-get update && apt-get install -y \
    $(cat packages.txt) 
RUN python3 -m pip install --upgrade pip	
RUN python3 -m pip install -r requirements.txt
RUN python3.10 -m pip install --upgrade pip
RUN python3.10 -m pip install -r requirements_3_10.txt
RUN conan config set general.revisions_enabled=1 && \
    conan profile new default --detect > /dev/null && \
    conan profile update settings.compiler.libcxx=libstdc++11 default
RUN rosdep update

# aliases
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /home/user/eyetracking_vs/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "alias die='tmux kill-server'" >> ~/.bashrc
RUN echo "alias robot_start='python3 ./startup/robot_start.py && tmux attach'" >> ~/.bashrc
RUN echo "alias source_all='source /opt/ros/$ROS_DISTRO/setup.bash && source catkin_ws/devel/setup.bash'" >> ~/.bashrc 
RUN echo "alias clean='rm -rf build && rm -rf devel'" >> ~/.bashrc
RUN echo "alias install_rosdeps='rosdep install --from-paths src --ignore-src -y'" >> ~/.bashrc
RUN echo "alias careful_make='rosdep install --from-paths src --ignore-src -y && catkin_make'"
RUN echo "alias gripper='rosrun kortex_bringup gripper.py'" >> ~/.bashrc
RUN echo "alias kortex_home='rosrun kortex_bringup send_gen3_home.py'" >> ~/.bashrc
RUN echo "alias jackal_home='rosrun kortex_bringup jackal_home.py'" >> ~/.bashrc
RUN echo "alias reset_all_tracking='rostopic pub --once /tracking_node/reset_all std_msgs/Empty'" >> ~/.bashrc
RUN echo "alias sim_start='python3 ./startup/sim_start.py && tmux attach'" >> ~/.bashrc
