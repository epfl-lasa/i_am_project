#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <cmath>

bool update = false;
bool prev_update = update;
void updateCallback(const std_msgs::Bool msg){
    update = msg.data;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "AH_trail_init");
    ros::NodeHandle nh;
    ros::Rate rate(100.0);

    ros::Publisher init_pub = nh.advertise<geometry_msgs::Pose>("trials/init",10);
    ros::Subscriber update_subs = nh.subscribe("trials/update",10,updateCallback);


    std::vector<double> centervec;
    std::vector<double> rangevec;
    int N;
    if(!nh.getParam("trials/center", centervec)){ROS_ERROR("Param trials/center not found");}
    if(!nh.getParam("trials/range", rangevec)){ROS_ERROR("Param trials/range not found");}
    if(!nh.getParam("trials/N", N)){ROS_ERROR("Param trials/N not found");}
    double center_x = centervec[0];
    double center_y = centervec[1];
    double range_x = rangevec[0];
    double range_y = rangevec[1];

    double N_x = sqrt(N*range_x/range_y);
    double N_y = sqrt(N*range_y/range_x);

    double dx = -0.5*range_x;
    double dy = -0.5*range_y;


    Eigen::Vector3d object_pos_init;
    geometry_msgs::Pose object_pose_init;

    int oma;
    while (nh.ok()) {
        if (update == true && prev_update == false){
            dx += range_x/N_x;
            if (dx > range_x/2){
                dx = -0.5*range_x;
                dy += range_y/N_y;
                if (dy > range_y/2){
                    dy = -0.5*range_y;
                }
            }
            
        }
        prev_update = update;

        object_pos_init << center_x + dx, center_y + dy, centervec[2];
        


        object_pose_init.position.x = object_pos_init[0];
        object_pose_init.position.y = object_pos_init[1];
        object_pose_init.position.z = object_pos_init[2];
        object_pose_init.orientation.w = 0;
        object_pose_init.orientation.x = 0;
        object_pose_init.orientation.y = 0;
        object_pose_init.orientation.z = 0;

        init_pub.publish(object_pose_init);

        ros::spinOnce();
        rate.sleep();
    }
}