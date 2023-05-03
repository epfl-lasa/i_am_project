ARG BASE_IMAGE_TAG=noetic
FROM epfl-lasa/iiwa_ros

# Modify files inside docker
ARG HOST_GID=1001
ENV USER_GROUP=${USER}
USER root
RUN if [ "${HOST_GID}" != "1000" ];\
    then groupadd --gid ${HOST_GID} host_group && \
    usermod ${USER} -g ${HOST_GID} && \ 
    usermod ${USER} -a -G ${USER_GROUP}; fi
USER ${USER}

# CPP lib
# osqp
WORKDIR /source
RUN git clone --recursive https://github.com/osqp/osqp
RUN cd osqp && mkdir build && cd build && cmake -G "Unix Makefiles" .. && cmake --build . && sudo cmake --build . --target install

# osqp-eigen
WORKDIR /source
RUN git clone https://github.com/robotology/osqp-eigen.git
RUN cd osqp-eigen && mkdir build && cd build && cmake ../ && make && sudo make install

# qpoases v3.2.1
WORKDIR /source
RUN git clone https://github.com/coin-or/qpOASES.git --branch releases/3.2.1
RUN cd qpOASES && mkdir build && cd build && cmake ../ && make && sudo make install

# iiwa_toolkit
ARG IIWA_TOOLKIT_BRANCH=feature_inertial_control
WORKDIR /home/${USER}/ros_ws/src
RUN git clone https://github.com/fkhadivar/iiwa_toolkit.git --branch $IIWA_TOOLKIT_BRANCH
# feature_inertial_control feature_full_inertia


# Copy i_am_project folder inside docker
WORKDIR /home/${USER}/ros_ws
COPY --chown=${USER} ./ ./src/i_am_project

ENV PYTHONPATH "${PYTHONPATH}:/opt/openrobots/lib/python3.8/site-packages/"
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin_make"

# ./docker/build-server.sh 
# aica-docker interactive iam_project_harshit:noetic -u ros --net host --no-hostname -v /home/elise/Documents/IAM/code/i_am_project:/home/ros/ros_ws/src/i_am_project


