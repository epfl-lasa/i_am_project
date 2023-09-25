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

# Add vrpn tools and rqt
RUN sudo apt-get update && sudo apt-get install -y \
    netbase \
    ros-noetic-vrpn-client-ros \
    ros-noetic-rqt-reconfigure \
    && sudo apt-get upgrade -y && sudo apt-get clean


# CPP lib
# osqp
WORKDIR /source
RUN git clone --recursive -b v0.6.3 https://github.com/osqp/osqp 
RUN cd osqp && mkdir build && cd build && cmake -G "Unix Makefiles" .. && cmake --build . && sudo cmake --build . --target install

# osqp-eigen
WORKDIR /source
RUN git clone https://github.com/robotology/osqp-eigen.git
RUN cd osqp-eigen && mkdir build && cd build && cmake ../ && make -j && sudo make install

# qpoases v3.2.1 --branch releases/3.2.1
WORKDIR /source
RUN git clone https://github.com/coin-or/qpOASES.git 
RUN cd qpOASES && mkdir build && cd build && cmake .. -DBUILD_SHARED_LIBS=True && make -j && sudo make install

# Install waf-tool (require to install optimization-lib)
WORKDIR /source
RUN git clone https://github.com/nash169/waf-tools.git
RUN cd waf-tools && sudo pip3 install .

# Install optimization-lib
RUN sudo apt-get install libnlopt-dev -y
WORKDIR /source
RUN git clone https://github.com/nash169/optimization-lib.git
RUN cd optimization-lib && sudo waf distclean configure build --shared install

# iiwa_toolkit (need to be root to clone private repo)
USER root
ARG IIWA_TOOLKIT_BRANCH=feature_ns_inertial_control
WORKDIR /home/${USER}/ros_ws/src
RUN --mount=type=ssh git clone --recursive git@github.com:epfl-lasa/iiwa_toolkit_ns.git --branch ${IIWA_TOOLKIT_BRANCH} 
# feature_ns_inertial_control feature_ns_full_inertia
USER ${USER}

# i_am_predict (kalman filter) (need to be root to clone private repo)
USER root
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


