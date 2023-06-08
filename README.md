
# i_am_project
## Dependencies/branches


### Inertia
* iiwa_ros - branch feature/double-robot-inertia - https://github.com/epfl-lasa/iiwa_ros/tree/feature/double-robot-inertia
* iiwa_toolkit - branch feature_ns_inertial_control - https://github.com/Elise-J/iiwa_toolkit/tree/feature_ns_inertial_control
* i_am_predict - branch master - https://github.com/epfl-lasa/i_am_predict/tree/master


### No inertia
* iiwa_ros - branch featuresim - https://github.com/epfl-lasa/iiwa_ros/tree/featuresim 
* iiwa_toolkit_ns - branch feature_AH - https://github.com/epfl-lasa/iiwa_toolkit_ns/tree/feature_AH 
* i_am_predict - branch master - https://github.com/epfl-lasa/i_am_predict/tree/master


<!-- |   | Single arm only hitting DS | Single arm with air hockey policy | Dual arm with air hockey policy |
| ------|-----|-----|-----|
| iiwa_ros | master | featuresim | featuresim |
| iiwa_toolkit	| master | - |-|
| iiwa_toolkit_ns	| - | feature_AH |feature_AH|
| i_am_project	| master | feature_real-sim_single |feature_real|
| i_am_predict	| - | - |master| -->

## launch: 
There are three different settings in which the branches with air hockey policy can be launched:
(The parameters can be changed in `i_am_project/config/world_sim_params.yaml`)
### Entirely in simulation
* Set parameter `object_real` to `false`
* Set parameter `iiwa_real` to `false`
* `roslaunch i_am_project air_hockey_gazebo.launch`

### Simulated IIWA's while using real object - NO TESTED YET
* Set parameter `object_real` to `true`
* Set parameter `iiwa_real` to `false`
* `roslaunch i_am_project air_hockey_real_gazebo.launch`

**Remark**: the pose of the box is tracked with Optitrack and used in the control program. NOT TESTED YET

### Entirely physical setup
You'll need 2 computer (you can use SSH connection)

#### Setup ROS communication between 2 computers
This must be run inside docker if used.

Add those line in `~/.bashrc` (To modify it `sudo nano ~/.bashrc`)
* Remote computer : 
``` bash 
export ROS_MASTER_URI=http://<ip-address-of-current-computer>:11311
export ROS_IP=<ip-address-of-remote-computer>  
```
* Current computer : 

``` bash 
export ROS_MASTER_URI=http://<ip-address-of-current-computer>:11311
export ROS_IP=<ip-address-of-current-computer>
```
Source the file when it's done (`. ~/.bashrc`)

#### Launch air hockey game (entirely physical setup)
1. `roslaunch i_am_project optitrack.launch` (Current computer)
2. `roslaunch iiwa_toolkit passive_track_real.launch robot_name:=iiwa2 model:=14` (computer connect to iiwa 14)
3. `roslaunch iiwa_toolkit passive_track_real.launch robot_name:=iiwa1 model:=7` (computer connect to iiwa 7)
4. Set parameter `object_real` to `true` and `iiwa_real` to `true` (file in `i_am_project/config/world_sim_params.yaml`) (Current computer)
5. `roslaunch i_am_project air_hockey_real.launch` (Current computer)


**Remark**:
* Tracking gains in iiwa_toolkit_ns/config/passuve_track_params.yaml might need tuning 
* If using inertia, ee_pose topic must be changed in config/ros_topics.yaml from `/iiwa2/ee_pose` to `/iiwa2/ee_info/Pose`


## Other settings:

* To add manual control, change parameter `manual` to `true` in the parameter file and run in a separate terminal: `rosrun i_am_project key_ctrl`. 
(Each time, press a couple of times to make sure the command is sent.)
	* Press '1' to initiate a switch for iiwa1 to track or hitting.
	* Press '2' to do the same for iiwa2.
	* Press '3' to switch both arms to rest. 
* To use hollow box, change parameter `hollow` to `true` in parameter file

Keep in mind for the single setup, the IIWA base is at (x,y)=(0,0) while for the dual setup, the point between the two IIWA bases is at (x,y)=(0,0).
Finally, it is important to set the center points in the parameter file to reasonable positions w.r.t. the IIWA_base. A good first option is 0.2 m in the direction of the other IIWA and 0.55 m perpendicular to that direction.



## Docker
The Dockerfile has the "Dual arm with air hockey policy" dependencies.

A docker containing iiwa-ros library is needed to build the i_am_project docker.

### Prerequisite

cf. https://github.com/epfl-lasa/iiwa_ros/tree/feature/dockerise/docker#prerequisite

### Docker iiwa-ros
#### Inertia
1. Pull the repo 
    ```bash
    git clone -b feature/double-robot-inertia git@github.com:epfl-lasa/iiwa_ros.git
    ```
    
2. Build the docker
    ``` bash
    cd docker
    bash install_docker.sh
    ```

#### No inertia
1. Pull the repo 
    ```bash
    git clone -b feature/dockerise git@github.com:epfl-lasa/iiwa_ros.git
    ```
    The branch of iiwa_ros needed here is `featuresim` . However, it needs to pull the changes on main to work properly.

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

    * To have the changes from branch `featuresim` simply run
    ``` bash
    git pull origin featuresim
    ```
    and resolve the merge conflicts **BUT DO NOT PUSH THE CHANGES** 

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
The files from your folder i_am_project will be copy inside the docker. Make sure you are on the correct branch.

**Build docker Inertia**
The branch of iiwa-toolkit lib can be chosen. The default branch is feature_ns_inertial_control

```bash
cd <path_to_i_am_project>

 ./docker/build-server.sh -i -b <iiwa-toolkit-branch-name>
```

**Build docker No inertia**

```bash
cd <path_to_i_am_project>

 ./docker/build-server.sh 
```

**Run docker**

``` bash 
aica-docker interactive iam_project_harshit:noetic -u ros --net host --no-hostname -v /path_to_project/i_am_project:/home/ros/ros_ws/src/i_am_project --privileged
```