# i_am_project

## Requirements
i_am_project requires several packages to be installed in order to work properly:

Note that it is possible to have everything on docker (cf. docker section)

* **iiwa_ros**: branch `feature/inertia` (and all its dependencies): https://github.com/epfl-lasa/iiwa_ros/tree/feature/inertia
* **iiwa_toolkit**: branch `feature_ns_inertial_control` or `feature_ns_full_inertia` (and their dependencies):  https://github.com/epfl-lasa/iiwa_toolkit_ns

if iiwa_toolkit branch `feature_ns_full_inertia` is used:

* **osqp**: https://github.com/osqp/osqp
* **osqp-eigen**: https://github.com/robotology/osqp-eigen.git
* **qpoases**: https://github.com/coin-or/qpOASES.git (works with v3.2.1)
* **waf-tool**: (required to install the optimization-lib): https://github.com/nash169/waf-tools
* **optimization-lib**: https://github.com/nash169/optimization-lib.git 

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
3. Log in to the docker registry with the credentials you have to login in gitlab 
```bash 
sudo docker login registry.algoryx.se
```

#### Get custom AGX scene

1. Navigate to the _Project_ folder
```bash
cd urdf-application/PythonApplication/models/Projects
```
2. Clone the repo containing the custom AGX scene
```bash
git clone https://github.com/epfl-lasa/agx_scene
```

#### Setup environment
Tested with python 3.8.10
1. `cd i_am_project && pip install -r requirements_agx.txt`
2. Install RBDyn
```bash 
RUN git clone --recursive https://github.com/jrl-umi3218/RBDyn.git
RUN cd RBDyn && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF .. \
    && make -j && sudo make install
```
3. Install mc_rbdyn_urdf
```bash 
RUN git clone --recursive -b v1.1.0 https://github.com/jrl-umi3218/mc_rbdyn_urdf.git
RUN cd mc_rbdyn_urdf && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF .. \
    && make -j && sudo make install
```
4. Add the cpp lib and python bindings to you path 
```bash 
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path_to_i_am_project/python_binding/build/' >> ~/.bashrc
. ~/.bashrc
sudo ldconfig
```

If the libraries of i_am_project or iiwa_toolkit need to be build:
```bash
 cd python_binding && mkdir build && cd build && cmake .. && make -j
```


#### Start simulation

With *iiwa_toolkit* branch *feature_ns_inertial_control*
1. Start AGX simulation
```bash
sudo python3 ../run-in-docker.py python3 click_application.py --model models/Projects/agx_scene/Scenes/IiwaClickScene.yml:IiwaTorqueClick --timeStep 0.005 --agxOnly --rcs --portRange 5656 5658  --disableClickSync
```
2. The simulation can be seen at  `http://localhost:5656/`
3. Start the controller:
```bash 
python3 script/python_agx_passive_inertial_control.py
```

With *iiwa_toolkit* branch *feature_ns_full_inertia*
1. Start AGX simulation
```bash
sudo python3 ../run-in-docker.py python3 click_application.py --model models/Projects/agx_scene/Scenes/IiwaClickScene.yml:IiwaAngleClick --timeStep 0.005 --agxOnly --rcs --portRange 5656 5658  --disableClickSync
```
2. The simulation can be seen at  `http://localhost:5656/`
3. Start the controller: 
```bash
python3 script/python_agx_full_inertia_control.py
```


## Docker

A docker containing iiwa-ros library is needed to build the i_am_project docker.

### Prerequisite

cf. https://github.com/epfl-lasa/iiwa_ros/tree/feature/dockerise/docker#prerequisite


### Docker iiwa-ros
1. Pull the repo 
    ```bash
    git clone -b feature/double-robot-inertia git@github.com:epfl-lasa/iiwa_ros.git
    ```
    
2. Build the iiwa-ros docker
    ``` bash
    cd docker
    bash install_docker.sh
    ```

### Docker i_am_project
The iiwa-ros docker needs to build first.

Build docker:
The branch of iiwa-toolkit lib can be chosen. The default branch is feature_inertial_control

```bash
 ./docker/build-server.sh -b <iiwa-toolkit-branch-name>
```

Run docker:

``` bash 
aica-docker interactive iam_project_harshit:noetic -u ros --net host --no-hostname -v /path_to_project/i_am_project:/home/ros/ros_ws/src/i_am_project 
```

# AIRHOCKEY SETUP REAL

``` bash 
aica-docker interactive iam_project_harshit:noetic -u ros --net host --no-hostname -v /home/maxime/Documents/i_am_project:/home/ros/ros_ws/src/i_am_project
```

Computer 1 - IP: 128.178.145.165 -> connected to iiwa7 (= iiwa1 = iiwa left)
Computer 2 - IP: 128.178.96.187 -> connected to iiwa14(= iiwa2 = iiwa right)

Connect via ssh from computer 1 to computer 2 : 
```bash
ssh elise@128.178.96.187
```

Launch docker on both computers with aica-docker command above or using the alias :
```bash
airhockey
```bash

Connect with other terminals using : 
```bash
aica-docker connect iam_project_harshit-noetic-runtime -u ros
```
(use this command on computer 2 to leave container running)

You should have 4 terminals, 3 connected to the docker container on computer 1 and one on the docker container on Computer 2. From now on, Computer 2 will refer to that last terminal (connected via ssh)

## Setup ROS communications

Modify ROS_MASTER_URI and ROS_IP inside both containers for proper ROS communications

both : 
```bash
sudo nano ~/.bashrc
```
Computer 1 : 
```bash
export ROS_MASTER_URI=http://128.178.145.165:11311 export ROS_IP=128.178.145.165
```
Computer 2 : 
```bash
export ROS_MASTER_URI=http://128.178.145.165:11311 export ROS_IP=128.178.96.187
```
both : 
```bash
source ~/.bashrc
```
## ROS launch commands

Launch the following commands in this order

Computer 1 - Terminal 1 :
```bash
roslaunch i_am_project optitrack.launch
```
Computer 2 - Terminal 2 :
```bash
roslaunch iiwa_toolkit passive_track_real.launch robot_name:=iiwa2 model:=14
```
Computer 1 - Terminal 3 :
```bash
roslaunch iiwa_toolkit passive_track_real.launch robot_name:=iiwa1 model:=7
```
Computer 1 - Terminal 4 :
```bash
roslaunch i_am_project air_hockey_real.launch
```

Sequence for iiwa trackpad :
iiwa7 : FRIOverlay -> Torque, 150, 0.6 (50,0.5)
iiwa14 : FRIOverlay -> Torque, 300, 0.2 

### Remarks and troubleshooting

Both computers must have the IP adress 190.170.10.1 for the conenction to the iiwa as FRIOverlay communicates only with this adress.
Can modify the IP adress of the robot in iiwa_driver/config/iiwa.yaml

Use 'sudo chmod 777 -R .' when having trouble saving files inside container 


