# i_am_project

i_am_project - 

## Docker

Change iiwa_toolkit branch in Dockerfile line 32

Build docker:
The branch of iiwa-toolkit lib can be chosen. The default branch is feature_inertial_control

```bash
 ./docker/build-server.sh -b <iiwa-toolkit-branch-name>
```

Run docker:

``` bash 
aica-docker interactive iam_project_harshit:noetic -u ros --net host --no-hostname -v /path_to_project/i_am_project:/home/ros/ros_ws/src/i_am_project
```

