# i_am_project

## Requirements
i_am_project requires several packages to be installed in order to work properly:

Note that it is possible to have everything on docker (cf. docker section)

* iiwa_ros - branch `feature/inertia` (and all its dependencies): https://github.com/epfl-lasa/iiwa_ros/tree/feature/inertia
* iiwa_toolkit - branch `feature_inertial_control` or `feature_full_inertia` (and all its dependencies):  https://github.com/fkhadivar/iiwa_toolkit
* osqp - https://github.com/osqp/osqp
* osqp-eigen - https://github.com/robotology/osqp-eigen.git
* qpoases v3.2.1 - https://github.com/coin-or/qpOASES.git

if iiwa_toolkit branch `feature_full_inertia` is used:

* waf-tool (required to install the optimization-lib): https://github.com/nash169/waf-tools
* optimization-lib: https://github.com/nash169/optimization-lib.git 

## Run the simulation

In one terminal, launch the gazebo simulation:
``` bash
roslaunch iiwa_toolkit passive_track_gazebo.launch
```
In another terminal, setup the world:
```bash
roslaunch i_am_project spawn_env.launch
```
Wait for the script to finish and launch the simulation:
```bash
roslaunch i_am_project hit_with_momentum.launch 
```


## Docker

A docker containing iiwa-ros library is needed to build the i_am_project docker.

### Prerequisite

cf. https://github.com/epfl-lasa/iiwa_ros/tree/feature/dockerise/docker#prerequisite


### Docker iiwa-ros

1. Pull the repo 
    ```bash
    git clone -b feature/dockerise git@github.com:epfl-lasa/iiwa_ros.git
    ```
    The branch of iiwa_ros needed here is `feature/inertia` . However, it needs to pull the changes on main to work properly.

    **When the pull will be done:**

    In Dockerfile located in ./docker replace (line 114)
    ```bash
    RUN git clone https://github.com/epfl-lasa/iiwa_ros.git
    ```
    with 
    ``` bash
    RUN git clone -b feature/inertia https://github.com/epfl-lasa/iiwa_ros.git
    ```

    **Quick fix when waiting for the pull:**

    * To have the changes from branch `feature/inertia` simply run
    ``` bash
    git pull origin feature/inertia
    ```
    and resolve the merge conflict **BUT DO NOT PUSH THE CHANGES**

    * In Dockerfile located in ./docker replace (line 114)
    ```bash
    RUN git clone https://github.com/epfl-lasa/iiwa_ros.git
    ```
    with 
    ``` bash
    COPY --chown=${USER} ./ ./src/iiwa_ros
    ```

2. Build the docker
    ``` bash
    cd docker
    bash install_docker.sh
    ```

### Docker i_am_project

Build docker:
The branch of iiwa-toolkit lib can be chosen. The default branch is feature_inertial_control

```bash
 ./docker/build-server.sh -b <iiwa-toolkit-branch-name>
```

Run docker:

``` bash 
aica-docker interactive iam_project_harshit:noetic -u ros --net host --no-hostname -v /path_to_project/i_am_project:/home/ros/ros_ws/src/i_am_project
```



