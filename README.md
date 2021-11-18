# i_am_project
### For single arm setup:
* iiwa_ros      -> master
* iiwa_toolkit  -> master
* i_am_project  -> featuresim_single

### For dual arm setup:
* iiwa_ros              -> featuresim
* dual_iiwa_toolkit     -> featuresim
* i_am_project          -> featuresim

## launch: 

`roslaunch i_am_project 1v1_Gazebo.launch`


## Change parameters for environment:
    
[i_am_project/config/1v1_params.yaml]

Keep in mind for the single setup, the iiwa base is at (x,y)=(0,0) while for the dual setup, the center of the table is at (x,y)=(0,0)