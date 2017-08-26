FROM ros:kinetic

RUN apt-get -qq update && \
  apt-get install -y --no-install-recommends sudo wget curl && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/*

RUN rosdep update && \
  mkdir -p /catkin_ws/src && \
  bash -c "cd /catkin_ws/src && . /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace && cd .. && catkin_make"

ARG TRAVIS_PULL_REQUEST=false
ARG TRAVIS_REPO_SLUG=""
ARG TRAVIS_BOT_GITHUB_TOKEN=""

ENV TRAVIS_PULL_REQUEST=$TRAVIS_PULL_REQUEST
ENV TRAVIS_REPO_SLUG=$TRAVIS_REPO_SLUG
ENV TRAVIS_BOT_GITHUB_TOKEN=$TRAVIS_BOT_GITHUB_TOKEN

COPY ./ /catkin_ws/src/hokuyo3d
RUN /catkin_ws/src/hokuyo3d/.travis.scripts/test.sh



