#include "init_world_sim.h"

//Some standard sdf bits which will be reused for each sdf file
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
                    </visual>)";

//Generate sdf files
std::stringstream sdfGenerator(const modelProperties model) {
  // this is the meticulous bit: create a model sdf code for the spawner by adding sections together such that the variables can be squeezed inbetween
  std::stringstream name_ss;
  std::stringstream com_ss;
  std::stringstream mass_ss;
  std::stringstream size_ss;
  std::stringstream mu_ss;
  std::stringstream sdf;

  name_ss << model.name << "\"> <pose>0 0 0 0 0 0 </pose> <static>false</static> <link name=\"" << model.name;
  com_ss << model.com_x << " " << model.com_y << " " << model.com_z << " 0 0 0 </pose> <mass>";
  mass_ss << model.mass << "</mass>  <inertia> <ixx>" << model.ixx << "</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz> <iyy>"
          << model.iyy << "</iyy> <iyz>0.0</iyz> <izz>" << model.izz;
  size_ss << model.size_x << " " << model.size_y << " " << model.size_z;
  mu_ss << model.mu << "</mu>  <mu2>" << model.mu2;

  if (model.name != "my_table") {
    sdf << sdf1 << name_ss.str() << sdf2 << com_ss.str() << mass_ss.str() << sdf3 << size_ss.str() << sdf4
        << mu_ss.str() << sdf5 << size_ss.str() << sdf6 << "</link> </model> </sdf>";
  } else {// table gets a border so the sdf code has to be extended
    std::stringstream name_ss;
    std::stringstream part1;
    std::stringstream border1;
    std::stringstream part2;
    std::stringstream border2;
    std::stringstream part3;
    std::stringstream border3;
    std::stringstream part4;
    std::stringstream border4;

    double extr = 0.02;

    name_ss << model.name << "\"> <pose>0 0 0 0 0 0 </pose> <static>true</static> <link name=\"" << model.name;
    part1 << " <pose>" << -model.size_x / 2 - 0.01 << " " << 0 << " " << (model.size_z) / 2 - 0.015
          << " 0 0 0</pose> <geometry> <box> <size> " << 0.02 << " " << model.size_y + 0.04 << " "
          << model.size_z + extr << "</size> </box> </geometry> ";
    border1 << "<collision name=\"collision1\">" << part1.str() << "</collision> <visual name=\"visual1\">"
            << part1.str() << "</visual>";
    part2 << " <pose>" << model.size_x / 2 + 0.01 << " " << 0 << " " << (model.size_z) / 2 - 0.015
          << " 0 0 0</pose> <geometry> <box> <size> " << 0.02 << " " << model.size_y + 0.04 << " "
          << model.size_z + extr << "</size> </box> </geometry> ";
    border2 << "<collision name=\"collision2\">" << part2.str() << "</collision> <visual name=\"visual2\">"
            << part2.str() << "</visual>";
    part3 << " <pose>" << 0 << " " << -model.size_y / 2 - 0.01 << " " << (model.size_z) / 2 - 0.015
          << " 0 0 0</pose> <geometry> <box> <size> " << model.size_x << " " << 0.02 << " " << model.size_z + extr
          << "</size> </box> </geometry> ";
    border3 << "<collision name=\"collision3\">" << part3.str() << "</collision> <visual name=\"visual3\">"
            << part3.str() << "</visual>";
    part4 << " <pose>" << 0 << " " << model.size_y / 2 + 0.01 << " " << (model.size_z) / 2 - 0.015
          << " 0 0 0</pose> <geometry> <box> <size> " << model.size_x << " " << 0.02 << " " << model.size_z + extr
          << "</size> </box> </geometry> ";
    border4 << "<collision name=\"collision4\">" << part4.str() << "</collision> <visual name=\"visual4\">"
            << part4.str() << "</visual>";

    sdf << sdf1 << name_ss.str() << sdf2 << com_ss.str() << mass_ss.str() << sdf3 << size_ss.str() << sdf4
        << mu_ss.str() << sdf5 << size_ss.str() << sdf6 << border1.str() << border2.str() << border3.str()
        << border4.str() << "</link> </model> </sdf>";
  }

  return sdf;
};

modelProperties
get_param(modelProperties model, std::string key, ros::NodeHandle nh, modelProperties box = modelProperties()) {
  nh.getParam(key + "/properties/size/x", model.size_x);
  nh.getParam(key + "/properties/size/y", model.size_y);
  nh.getParam(key + "/properties/size/z", model.size_z);
  nh.getParam(key + "/properties/COM/x", model.com_x);
  nh.getParam(key + "/properties/COM/y", model.com_y);
  nh.getParam(key + "/properties/COM/z", model.com_z);
  nh.getParam(key + "/properties/mass", model.mass);
  nh.getParam(key + "/properties/mu", model.mu);
  nh.getParam(key + "/properties/mu2", model.mu2);

  model.ixx = 1.0 / 12 * model.mass * (pow(model.size_y, 2) + pow(model.size_z, 2));
  model.iyy = 1.0 / 12 * model.mass * (pow(model.size_x, 2) + pow(model.size_z, 2));
  model.izz = 1.0 / 12 * model.mass * (pow(model.size_x, 2) + pow(model.size_y, 2));

  nh.getParam(key + "/initial_pos/x", model.pose.position.x);
  nh.getParam(key + "/initial_pos/y", model.pose.position.y);
  nh.getParam(key + "/initial_pos/z", model.pose.position.z);

  if (key == "mini") {
    model.pose.position.x += box.pose.position.x;
    model.pose.position.y += box.pose.position.y;
    model.pose.position.z += box.pose.position.z;
  }
  model.pose.orientation.x = 0.0;
  model.pose.orientation.y = 0.0;
  model.pose.orientation.z = 0.0;
  model.pose.orientation.w = 1.0;

  return model;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "init_world_sim");
  ros::NodeHandle nh;

  //Spawn service
  ros::service::waitForService("gazebo/spawn_sdf_model");
  ros::ServiceClient model_spawner = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
  gazebo_msgs::SpawnModel model_spawn;

  bool hollow;
  nh.getParam("hollow", hollow);

  //Get parameters for the models
  //Table
  struct modelProperties table;
  table.name = "my_table";
  table = get_param(table, "table", nh);

  //Box:
  struct modelProperties box;
  box.name = "my_box";
  box = get_param(box, "box", nh);

  //Send info to gazebo
  //Table:
  model_spawn.request.model_name = table.name;
  model_spawn.request.model_xml = sdfGenerator(table).str();
  model_spawn.request.robot_namespace = "air_hockey_gazebo";
  model_spawn.request.initial_pose = table.pose;
  model_spawner.call(model_spawn);

  //Box:
  if (hollow == false) {
    model_spawn.request.model_name = box.name;
    model_spawn.request.model_xml = sdfGenerator(box).str();
    model_spawn.request.robot_namespace = "air_hockey_gazebo";
    model_spawn.request.initial_pose = box.pose;
  } else {//Hollow box is the only model that still uses fixed external sdf file
    std::stringstream hollow_path;
    hollow_path << ros::package::getPath("i_am_project") << "/object_models/hollow_box.sdf";

    std::ifstream hollow_box(hollow_path.str());

    if (!hollow_box) { std::cerr << "Unable to open hollow_box.txt"; }

    std::stringstream hollow_box_sdf;
    hollow_box_sdf << hollow_box.rdbuf();
    hollow_box.close();

    model_spawn.request.model_name = box.name;
    model_spawn.request.model_xml = hollow_box_sdf.str();
    model_spawn.request.robot_namespace = "air_hockey_gazebo";
    model_spawn.request.initial_pose = box.pose;
    model_spawner.call(model_spawn);

    //Mini box:
    struct modelProperties mini;
    mini.name = "my_mini";
    mini = get_param(mini, "mini", nh, box = box);

    model_spawn.request.model_name = mini.name;
    model_spawn.request.model_xml = sdfGenerator(mini).str();
    model_spawn.request.robot_namespace = "air_hockey_gazebo";
    model_spawn.request.initial_pose = mini.pose;
  }
  model_spawner.call(model_spawn);
}