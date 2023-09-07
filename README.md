# i_am_project

## Requirements
i_am_project requires several packages to be installed in order to work properly:

Note that it is possible to have everything on docker (cf. docker section)

* iiwa_ros - branch `feature/inertia` (and all its dependencies): https://github.com/epfl-lasa/iiwa_ros/tree/feature/inertia
* iiwa_toolkit - branch `feature_ns_inertial_control` or `feature_ns_full_inertia` (and all its dependencies):  https://github.com/fkhadivar/iiwa_toolkit TODO REPO IN EPFL-LASA
* osqp - https://github.com/osqp/osqp
* osqp-eigen - https://github.com/robotology/osqp-eigen.git
* qpoases https://github.com/coin-or/qpOASES.git (works with v3.2.1)

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

## Run the simulation with AGX

### Urdf-application
#### Get URDF-app
1. Clone the urdf-application in branch fix/model-structure  
```bash
git clone -b fix/model-structure git@git.algoryx.se:algoryx/external/i-am/urdf-application.git
```
2. Follow the docker installation steps here: https://git.algoryx.se/algoryx/external/i-am/urdf-application#installing-docker
3. Log in to the docker registry with the credentials you have to login in gitlab `sudo docker login registry.algoryx.se`

#### Get custom Project scene for IIWA
1. `cd urdf-application/PythonApplication/models/Projects`
2. Clone this repo: `git clone https://github.com/Elise-J/iam_sim_agx.git`

#### Setup environment
Tested with python 3.8.10
1. ` cd i_am_project && pip install -r requirements_agx.txt`
2. The repo `iiwa_toolkit` needs to be cloned next to `i_am_project` (or change the path in `i_am_project/script/python_agx_passive_inertial_control.py` and  `i_am_project/script/python_agx_full_inertial_control.py`, line 14)


#### Start simulation

With `iiwa_toolkit` branch `feature_ns_inertial_control`
1. Start AGX simulation `sudo python3 ../run-in-docker.py python3 click_application.py --model models/Projects/i_am_project/Scenes/IiwaClickScene.yml:IiwaTorqueClick --timeStep 0.005 --agxOnly --rcs --portRange 5656 5658  --disableClickSync`
2. Open your browser and go to `http://localhost:5656/`
3. Start the controller: `python3 script/python_agx_passive_inertial_control.py`

With `iiwa_toolkit` branch `feature_ns_full_inertia`
1. Start AGX simulation `sudo python3 ../run-in-docker.py python3 click_application.py --model models/Projects/i_am_project/Scenes/IiwaClickScene.yml:IiwaAngleClick --timeStep 0.005 --agxOnly --rcs --portRange 5656 5658  --disableClickSync`
2. Open your browser and go to `http://localhost:5656/`
3. Start the controller: `python3 script/python_agx_full_inertial_control.py`


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



