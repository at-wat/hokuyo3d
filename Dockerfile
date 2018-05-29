FROM ros:kinetic

RUN apt-get -qq update && \
  apt-get install -y --no-install-recommends sudo wget curl python-pip && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/*

RUN rosdep update && \
  mkdir -p /catkin_ws/src && \
  bash -c "cd /catkin_ws/src && . /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace && cd .. && catkin_make"

COPY ./ /catkin_ws/src/hokuyo3d
