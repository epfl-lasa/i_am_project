
# i_am_project
## Dependencies/branches
|   | Single arm only hitting DS | Single arm with air hockey policy | Dual arm with air hockey policy |
| ------|-----|-----|-----|
| iiwa_ros | master | featuresim | featuresim |
| iiwa_toolkit	| master | - |-|
| iiwa_toolkit_ns	| - | feature_AH |feature_AH|
| i_am_project	| master | feature_real-sim_single |feature_real|
| i_am_predict	| - | - |master|

**Note for single arm with air hockey policy**
Works with trial initializer node `1v1_trial_init.cpp` which publishes initial object position for each trial
TODO THIS NODE DOESNT EXISTS?!

## launch: 
There are three different settings in which the branches with air hockey policy can be launched:
(The parameters can be changed in `i_am_project/config/world_sim_params.yaml`)
### Entirely in simulation
* `roslaunch i_am_project air_hockey_gazebo.launch`
* Set parameter `object_real` to `false`
* Set parameter `iiwa_real` to `false`

### Simulated IIWA's while using real object
* `roslaunch i_am_project air_hockey_real_gazebo.launch`
* Set parameter `object_real` to `true`
* Set parameter `iiwa_real` to `false`

**Remark**: the pose of the box is tracked with Optitrack and used in the control program.

### Entirely physical setup
* `roslaunch i_am_project air_hockey_real.launch`
* Set parameter `object_real` to `true`
* Set parameter `iiwa_real` to `true`


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

Build docker:

```bash
cd <path_to_i_am_project>

 ./docker/build-server.sh 
```

Run docker:

``` bash 
aica-docker interactive iam_project_harshit:noetic -u ros --net host --no-hostname -v /path_to_project/i_am_project:/home/ros/ros_ws/src/i_am_project
```