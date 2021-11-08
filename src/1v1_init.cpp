#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "geometry_msgs/Pose.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>


const char* sdf1 = R"(<?xml version="1.0" ?>
    <sdf version="1.4">
        <model name=")";
                    //my_table"> <pose>0 0 0 0 0 0 </pose> <static>false</static> <link name="my_table
const char* sdf2 = R"(">
                <gazebo reference = "world">
                    <turnGravityOff>true</turnGravityOff>
                </gazebo>
                <inertial>
                    <pose>)";
                        //0 0 0 0 0 0 </pose> <mass>0.05</mass>  <inertia> <ixx>0.00033</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz> <iyy>0.00033</iyy> <iyz>0.0</iyz> <izz>0.00033
const char* sdf3 = R"(</izz> </inertia>
                </inertial>
                <collision name="collision">
                    <geometry>
                        <box> 
                            <size>)";
                                //0.1 0.1 0.1
const char* sdf4 = R"(          </size>
                        </box>
                    </geometry>
                    <surface>
                        <friction>
                            <ode>
                                <mu>)";
                                    //0.01</mu>  <mu2>0.01
const char* sdf5 = R"(              </mu2>
                            </ode>
                        </friction>
                    </surface>
                </collision>
                <visual name="visual">
                    <geometry>
                        <box> 
                            <size>)";
                                //0.1 0.1 0.1
const char* sdf6 = R"(          </size>
                            </box>
                        </geometry>
                    </visual>)"
;

struct modelProperties {
    const char* name;

    double size_x;
    double size_y;
    double size_z;
    double com_x;
    double com_y;
    double com_z;
    double mass;
    double mu;
    double mu2;

    double ixx;
    double iyy;
    double izz;

    geometry_msgs::Pose pose;
};

std::stringstream sdfGenerator(const modelProperties model){
    //this is the meticulous bit: create a model sdf code for the spawner by adding sections together such that the variables can be squeezed inbetween
    std::stringstream name_ss;
    std::stringstream com_ss;
    std::stringstream mass_ss;
    std::stringstream size_ss;
    std::stringstream mu_ss;
    name_ss << model.name << "\"> <pose>0 0 0 0 0 0 </pose> <static>false</static> <link name=\"" << model.name;
    com_ss << model.com_x << " " << model.com_y << " " << model.com_z << " 0 0 0 </pose> <mass>";
    mass_ss << model.mass << "</mass>  <inertia> <ixx>" << model.ixx << "</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz> <iyy>" << model.iyy << "</iyy> <iyz>0.0</iyz> <izz>" << model.izz;
    size_ss << model.size_x << " " << model.size_y << " " << model.size_z;
    mu_ss << model.mu << "</mu>  <mu2>" << model.mu2;

    std::stringstream sdf;
    if (model.name != "my_table") {
        sdf << sdf1 << name_ss.str() << sdf2 << com_ss.str() << mass_ss.str() << sdf3 << size_ss.str() << sdf4 << mu_ss.str() << sdf5 << size_ss.str() << sdf6 << "</link> </model> </sdf>";
    }
    else { //table gets a border so the sdf code has to be extended
        double extr = 0.05;
        std::stringstream part1;
        std::stringstream border1;
        std::stringstream part2;
        std::stringstream border2;
        /*std::stringstream part3;
        std::stringstream border3;
        std::stringstream part4;
        std::stringstream border4;*/
        part1 << " <pose>" << -model.size_x/2-0.01 << " " << 0 << " " << (model.size_z)/2 << " 0 0 0</pose> <geometry> <box> <size> " << 0.02 << " " << model.size_y << " " << model.size_z+extr << "</size> </box> </geometry> ";
        border1 << "<collision name=\"collision1\">" << part1.str() << "</collision> <visual name=\"visual1\">" << part1.str() << "</visual>";
        part2 << " <pose>" <<  model.size_x/2+0.01 << " " << 0 << " " << (model.size_z)/2 << " 0 0 0</pose> <geometry> <box> <size> " << 0.02 << " " << model.size_y << " " << model.size_z+extr << "</size> </box> </geometry> ";
        border2 << "<collision name=\"collision2\">" << part2.str() << "</collision> <visual name=\"visual2\">" << part2.str() << "</visual>";
        /*part3 << " <pose>" << 0 << " " << -model.size_y/2-0.01 << " " << (model.size_z+extr)/2 << " 0 0 0</pose> <geometry> <box> <size> " << model.size_x << " " << 0.02 << " " << model.size_z+extr << "</size> </box> </geometry> ";
        border3 << "<collision name=\"collision3\">" << part3.str() << "</collision> <visual name=\"visual3\">" << part3.str() << "</visual>";
        part4 << " <pose>" << 0 << " " <<  model.size_y/2+0.01 << " " << (model.size_z+extr)/2 << " 0 0 0</pose> <geometry> <box> <size> " << model.size_x << " " << 0.02 << " " << model.size_z+extr << "</size> </box> </geometry> ";
        border4 << "<collision name=\"collision4\">" << part4.str() << "</collision> <visual name=\"visual4\">" << part4.str() << "</visual>";*/
        sdf << sdf1 << name_ss.str() << sdf2 << com_ss.str() << mass_ss.str() << sdf3 << size_ss.str() << sdf4 << mu_ss.str() << sdf5 << size_ss.str() << sdf6 << border1.str() << border2.str() /*<< border3.str() << border4.str()*/ << "</link> </model> </sdf>";
    }
    
    return sdf;
};


int main (int argc, char** argv){
    ros::init(argc, argv, "1v1_init");
    ros::NodeHandle nh;
    //system("rosparam load $(rospack find i_am_1v1)/config/1v1_params.yaml");



    //spawn service
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient model_spawner = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel model_spawn;
    


    //get parameters for table
    struct modelProperties table;
        table.name = "my_table";
        nh.getParam("table/properties/size/x", table.size_x);
        nh.getParam("table/properties/size/y", table.size_y);
        nh.getParam("table/properties/size/z", table.size_z);
        nh.getParam("table/properties/COM/x", table.com_x);
        nh.getParam("table/properties/COM/y", table.com_y);
        nh.getParam("table/properties/COM/z", table.com_z);
        nh.getParam("table/properties/mass", table.mass);
        nh.getParam("table/properties/mu", table.mu);
        nh.getParam("table/properties/mu2", table.mu2);

        table.ixx = 1.0/12* table.mass* (pow(table.size_y,2) + pow(table.size_z,2));
        table.iyy = 1.0/12* table.mass* (pow(table.size_x,2) + pow(table.size_z,2));
        table.izz = 1.0/12* table.mass* (pow(table.size_x,2) + pow(table.size_y,2));
    
        nh.getParam("table/initial_pos/x", table.pose.position.x);
        nh.getParam("table/initial_pos/y", table.pose.position.y);
        nh.getParam("table/initial_pos/z", table.pose.position.z);
        table.pose.orientation.x = 0.0;
        table.pose.orientation.y = 0.0;
        table.pose.orientation.z = 0.0;
        table.pose.orientation.w = 1.0;
    
    //get parameters for box
    struct modelProperties box;
        box.name = "my_box";
        nh.getParam("box/properties/size/x", box.size_x);
        nh.getParam("box/properties/size/y", box.size_y);
        nh.getParam("box/properties/size/z", box.size_z);
        nh.getParam("box/properties/COM/x", box.com_x);
        nh.getParam("box/properties/COM/y", box.com_y);
        nh.getParam("box/properties/COM/z", box.com_z);
        nh.getParam("box/properties/mass", box.mass);
        nh.getParam("box/properties/mu", box.mu);
        nh.getParam("box/properties/mu2", box.mu2);

        box.ixx = 1.0/12* box.mass* (pow(box.size_y,2) + pow(box.size_z,2));
        box.iyy = 1.0/12* box.mass* (pow(box.size_x,2) + pow(box.size_z,2));
        box.izz = 1.0/12* box.mass* (pow(box.size_x,2) + pow(box.size_y,2));
        
        nh.getParam("box/initial_pos/x", box.pose.position.x);
        nh.getParam("box/initial_pos/y", box.pose.position.y);
        nh.getParam("box/initial_pos/z", box.pose.position.z);
        box.pose.orientation.x = 0.0;
        box.pose.orientation.y = 0.0;
        box.pose.orientation.z = 0.0;
        box.pose.orientation.w = 1.0;


    //finally add all information together in the service message (model_spawn) and call the service
    model_spawn.request.model_name = table.name;
    model_spawn.request.model_xml = sdfGenerator(table).str();
    model_spawn.request.robot_namespace = "1v1_Gazebo";
    model_spawn.request.initial_pose = table.pose;
    
    model_spawner.call(model_spawn); 

    //and for the box
    model_spawn.request.model_name = box.name;
    model_spawn.request.model_xml = sdfGenerator(box).str();
    model_spawn.request.robot_namespace = "1v1_Gazebo";
    model_spawn.request.initial_pose = box.pose;
    
    model_spawner.call(model_spawn); 
}