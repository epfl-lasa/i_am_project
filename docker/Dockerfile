ARG BASE_IMAGE_TAG=noetic
FROM epfl-lasa/iiwa_ros_feature_sim

# Modify files inside docker
ARG HOST_GID=1001
ENV USER_GROUP=${USER}
USER root
RUN if [ "${HOST_GID}" != "1000" ];\
    then groupadd --gid ${HOST_GID} host_group && \
    usermod ${USER} -g ${HOST_GID} && \ 
    usermod ${USER} -a -G ${USER_GROUP}; fi
USER ${USER}

# iiwa_toolkit (need to be root to clone private repo)
USER root
WORKDIR /home/${USER}/ros_ws/src
RUN --mount=type=ssh git clone -b feature_AH git@github.com:epfl-lasa/iiwa_toolkit_ns.git

# i_am_predict (kalman filter) (need to be root to clone private repo)
WORKDIR /home/${USER}/ros_ws/src
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/i_am_predict.git
RUN cd i_am_predict && git checkout clean/master

USER ${USER}

# Copy i_am_project folder inside docker
WORKDIR /home/${USER}/ros_ws
COPY --chown=${USER} ./ ./src/i_am_project

ENV PYTHONPATH "${PYTHONPATH}:/opt/openrobots/lib/python3.8/site-packages/"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make"

# ./docker/build-server.sh 
# aica-docker interactive iam_project_harshit:noetic -u ros --net host --no-hostname -v /home/elise/Documents/IAM/code/i_am_project:/home/ros/ros_ws/src/i_am_project


