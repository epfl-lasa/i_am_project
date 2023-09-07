#!/usr/bin/env python3

import numpy as np
import rospy
from dataclasses import dataclass

# AGX
from pclick import Client
from pclick import MessageFactory
from scipy.spatial.transform import Rotation as R

# Python wrapper lib
import sys
sys.path.append("python_binding_lib")
from py_wrap_dynamical_system import hitting_DS
from py_full_inertial_control import PassiveControl

# ------------ VARIABLE TO MODIFY ------------
dt = 0.001

# -- passive controller
urdf_path = "urdf/iiwa14.urdf"
end_effector = "iiwa_link_ee"
des_pos = np.array([0.5, -0.25, 0.3])
des_quat = np.array([0.7071068, -0.7071068 , 0.0,  0.0])
ds_gain_pos = 6.0
lambda0_pos = 70.0
lambda1_pos = 35.0
ds_gain_ori = 3.0
lambda0_ori = 5.0
lambda1_ori = 2.5

des_inertia = np.array([[4, 1, 1], [1, 3, 2], [1, 2, 3]])

# -- passive ds
hit_direction = [0, 1, 0]
iiwa_return_position = [0.5, -0.25, 0.3]

# --------------------------------------------


@dataclass
class Robot:
    joint_position = [0, 0, 0, 0, 0, 0, 0]
    joint_velocity = [0, 0, 0, 0, 0, 0, 0]
    joint_effort = [0, 0, 0, 0, 0, 0, 0]


@dataclass
class Object:
    position = [0, 0, 0]
    orientation = [0, 0, 0, 0]
    # object_veloctiy # TODO needed? if yes need compute


def generate_hitting_ee_vel(is_hit, hitting_generation, iiwa_task_inertia_pos):
    if not is_hit:
        ref_velocity_ee = hitting_generation.flux_DS(0.5, iiwa_task_inertia_pos)
    else:
        ref_velocity_ee = hitting_generation.linear_DS(iiwa_return_position)

    if not is_hit and np.dot(hitting_generation.get_des_direction(), (hitting_generation.get_DS_attractor() - hitting_generation.get_current_position())) < 0:
        is_hit = True
        
    return ref_velocity_ee, is_hit


def reset_sim_agx():
    # Send 0 velocity
    message = MessageFactory.create_controlmessage()
    robot_msg = message.objects["robot"]
    robot_msg.angles.extend([0, 0, 0, 0, 0, 0, 0])
    client.send(message)
    response = client.recv()
    # Reset Sim
    message = MessageFactory.create_resetmessage()
    client.send(message)
    response = client.recv()

def init_controller(robot):
    urdf_string = open(urdf_path).read()
    controller = PassiveControl(urdf_string, end_effector)
    controller.set_desired_inertia(des_inertia)
    controller.set_desired_pose(des_pos,des_quat)
    controller.set_pos_gains(ds_gain_pos,lambda0_pos,lambda1_pos)
    controller.set_ori_gains(ds_gain_ori,lambda0_ori,lambda1_ori)
    controller.updateRobot(robot.joint_position,robot.joint_velocity,robot.joint_effort)
    return controller

def init_hitting_ds(ee_pos, box):   
    hitting_controller =  hitting_DS(ee_pos, box.position)
    hitting_controller.set_des_direction(hit_direction)
    return hitting_controller

def get_agx_sensors(robot, box):
    message = MessageFactory.create_sensorrequestmessage()
    client.send(message)
    response = client.recv()

    robot.joint_position = np.array(response.objects['robot'].angleSensors) # CHECK IF RAD OR ANGLE
    robot.joint_velocity = np.array(response.objects['robot'].angleVelocitySensors)
    robot.joint_effort = np.array(response.objects['robot'].torqueSensors)

    box.position = response.objects['Box'].objectSensors[0].position.arr
    box_ori = response.objects['Box'].objectSensors[1].rpy.arr
    r = R.from_euler('xzy', [box_ori[0], box_ori[1], box_ori[2]], degrees=True)
    box.orientation = r.as_quat()

def send_command_agx(joint_position_des):
    message = MessageFactory.create_controlmessage()
    robot_msg_command = message.objects["robot"]
    robot_msg_command.angles.extend(list(joint_position_des))
    client.send(message)
    response = client.recv()

if __name__ == '__main__':

    # Connect to AGX sim
    addr = f"tcp://localhost:5555"
    client = Client()
    print(f"Connecting to click server {addr}")
    client.connect(addr)
    reset_sim_agx()

    # Init robot and object
    robot = Robot
    box = Object
    
    
    # Init passive controller and hitting DS
    get_agx_sensors(robot, box)
    controller = init_controller(robot)
    hitting_generation = init_hitting_ds(controller.getEEpos(), box)


    cycleCount = 0
    robot_init_pos = False
    max_cycle_init_pos = 10
    cycle_init_pos = 0
    is_hit = False

    while True:
        # Update controllers
        get_agx_sensors(robot, box)
        controller.updateRobot(robot.joint_position,robot.joint_velocity,robot.joint_effort)
        hitting_generation.set_current_position(controller.getEEpos())

        if np.linalg.norm(des_pos - controller.getEEpos()) < 0.001:
            cycle_init_pos = cycle_init_pos + 1
            if cycle_init_pos >= max_cycle_init_pos:
                robot_init_pos = True
        
        if robot_init_pos:
            ref_velocity_ee, is_hit = generate_hitting_ee_vel(is_hit, hitting_generation, controller.getTaskInertiaPos())
            controller.set_desired_velocity(ref_velocity_ee)

        # Compute joint position
        joint_position_desired = controller.computeJointPositionQP(dt)

        # Send command
        send_command_agx(joint_position_desired)

        cycleCount = cycleCount + 1

