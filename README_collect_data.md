

# i_am_project
# Data collection
All parameters can be changed in `i_am_project/config/world_sim_params.yaml`

## Launch: 
### Entirely in simulation
* Set parameter `object_real` to `false`
* Set parameter `iiwa_real` to `false`
* Run `roslaunch i_am_project air_hockey_gazebo_record.launch`

The nodes called in this launch file are `world_sim`, `air_hockey`, `estimate_sim` and `collect_data`:
-  `world_sim` initializes the air_hockey_gazebo simulation
-  `air_hockey` controls the different operation modes (1: track, 2: stop, 3: hit, 4: post-hit, 5: rest)
-  `estimate_sim` publishes the predicted stop pose of the object calculated using the real velocity of the object from Gazebo. To publish the predicted stop pose of the object calculated using the estimated velocity of the kalman filter, replace the node `estimate_sim` by `estimate_real` in the launch file `air_hockey_gazebo_record.launch`. Both nodes `estimate_sim` and `estimate_real` publish the predicted pose on the same topic `estimate/object`.
- The node `collect_data` collects and print data in `i_am_project/data/collect/object_data.csv`


### Entirely physical setup
The pose of the box is tracked with Optitrack and used in the control program.
* Set parameter `object_real` to `true`
* Set parameter `iiwa_real` to `true`
* Run a launch file  

**This set up is not tested with the recording of data yet.**
The relevent callback functions and prints are implemented in the file `collect_data.cpp` but there may be mistakes especially with the fact that Optitrack use another origin for its measurements. 

## How does it work?
We want to collect data of the object and the robot pre and post impact.

The pre-hit is only a few time steps before an event that we can recognize which is the mode 4 (post-hit) which happens as soon as the object has been hit.
Hence, we store the actual data for 3 time steps. This turns out to be the right time. In other words, post-hit mode is initiated 3 time steps after the step right before impact, which is the step we want data from.
The predicted stop pose is stored 1 second (100 time steps) before the pre-hit data are stored, you can compare the predicted stop pose of the object with the actual stop pose which is the pre-hit object pose. To store an earlier prediction, just change the number of time steps in lines 345 and 346 in the file `collect_data.cpp`. 

Note : the frequency is set to 100 so 100 time steps &rarr; 1 second.


## What is printed?
* `pre_post_time` (2 values in order) : `time_pre_hit_data`, `time_post_hit_data`

* `box_properties` (10 values in order) : `box.size_x`, `box.size_y`, `box.size_z`, `box.com_x`, `box.com_y`, `box.com_z`, `box.mass`, `box.mu`, `box.mu2`, `box.izz`

* `pre_hit_joints` (7 angles in order)  : pre_hit_iiwa_joint_angles [0..6]

* `post_hit_joints ` (7 angles in order)  : post_hit_iiwa_joint_angles [0..6]

* `pre_hit_ee` (16 values in order) : `ee_pose.position.x`, `ee_pose.position.y`, `ee_pose.position.z`, `ee_pose.orientation.x`, `ee_pose.orientation.y`, `ee_pose.orientation.z`, `ee_pose.orientation.w`, `ee_pose_roll`, `ee_pose_pitch`, `ee_pose_yaw`, `ee_linear_velocity_x`, `ee_linear_velocity_y`, `ee_linear_velocity_z`, `ee_angular_velocity_x`, `ee_angular_velocity_y`, `ee_angular_velocity_z` 

* `post_hit_ee` (16 values in order) : `ee_pose.position.x`, `ee_pose.position.y`, `ee_pose.position.z`, `ee_pose.orientation.x`, `ee_pose.orientation.y`, `ee_pose.orientation.z`, `ee_pose.orientation.w`, `ee_pose_roll`, `ee_pose_pitch`, `ee_pose_yaw`, `ee_linear_velocity_x`, `ee_linear_velocity_y`, `ee_linear_velocity_z`, `ee_angular_velocity_x`, `ee_angular_velocity_y`, `ee_angular_velocity_z` 

 * `pred_stop_pos` (4 values in order)  : `pred_time`, `pred_object_pose_x`, `pred_object_pose_y`, `pred_object_pose_z`

* `pre_hit_object` (9 values in order)  : `time_pre_hit`, `object_pose_x`, `object_pose_y`, `object_pose_z`, `object_theta`, `object_vel_x`, `object_vel_y`, `object_vel_z`, `object_theta_dot`

* `post_hit_object` (9 values in order)  : `time_pre_hit`, `object_pose_x`, `object_pose_y`, `object_pose_z`, `object_theta`, `object_vel_x`, `object_vel_y`, `object_vel_z`, `object_theta_dot`

* `post_hit_trajectory` Next, we track the pose of the object after the hit. Different conditions can stop the tracking. But until the tracking is stopped we print:

(9 values in order)  : `current_time`, `object_pose_x`, `object_pose_y`, `object_pose_z`, `object_theta`, `object_vel_x`, `object_vel_y`, `object_vel_z`, `object_theta_dot`

## When the tracking is stopped?
1.  Velocity of the object becomes 0 &rarr; tracking stopped with end message "end, free v=0 ". Doesn't happen often.
3. Most of the time, the object is hit back by the other iiwa robot before it comes to a complete halt. tracking stopped when the other robot enters the hit mode (mode 3) with end message "end, free m3"
4.  In case the predicted end position of the object trajectory is beyond the work-space in which hitting is still feasible, the arm should act to stop the object in its trajectory before reaching this position. In this case, the tracking is stopped when the other robot enters the stop mode (mode 2) with end message "end, stopped".
5. Although we have the stop mode, the object can still go out of reach &rarr; tracking stopped with end message "end, reset".
6. The user choose to exit the simulation &rarr; tracking stopped with end message "end, reset".

Note: There is still one case not taken into consideration which happens when the simulation "crashes" i.e. the robot's arm stops moving. In this case, the simplest way to deal with it is to stop the simulation and manually delete the last recorded data.


