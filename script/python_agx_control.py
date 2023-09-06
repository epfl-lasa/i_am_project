#!/usr/bin/env python3

import numpy as np
import rospy
from dataclasses import dataclass

# AGX
from pclick import Client
from pclick import MessageFactory
from scipy.spatial.transform import Rotation as R
# from roboticstoolbox.robot.ERobot import ERobot

# Python wrapper lib
import sys
sys.path.append("python_binding_lib")
from py_wrap_passive_control import PassiveControl

# ------------ VARIABLE TO MODIFY ------------
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


def generate_hitting():
    print("GENERATE HITTING - TODO")


def reset_sim_agx():
    # Send 0 velocity
    message = MessageFactory.create_controlmessage()
    robot_msg = message.objects["robot"]
    robot_msg.torques.extend([0, 0, 0, 0, 0, 0, 0])
    client.send(message)
    response = client.recv()
    # Reset Sim
    message = MessageFactory.create_resetmessage()
    client.send(message)
    response = client.recv()

def init_controller():
    urdf_string = open(urdf_path).read()
    controller = PassiveControl(urdf_string, end_effector)
    controller.set_desired_pose(des_pos,des_quat)
    controller.set_pos_gains(ds_gain_pos,lambda0_pos,lambda1_pos)
    controller.set_ori_gains(ds_gain_ori,lambda0_ori,lambda1_ori)
    return controller

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

def send_command_agx(command):
    message = MessageFactory.create_controlmessage()
    robot_msg_command = message.objects["robot"]
    robot_msg_command.torques.extend(list(command))
    client.send(message)
    response = client.recv()

if __name__ == '__main__':

    # Connect to AGX sim
    addr = f"tcp://localhost:5555"
    client = Client()
    print(f"Connecting to click server {addr}")
    client.connect(addr)
    reset_sim_agx()
    
    # Init the passive controller
    controller = init_controller()

    # Init robot and object
    robot = Robot
    box = Object

    cycleCount = 0
    robot_init_pos = False
    max_cycle_init_pos = 10
    cycle_init_pos = 0

    while True:
        # Get joint + box states
        get_agx_sensors(robot, box)
        
        if np.linalg.norm(des_pos - controller.getEEpos()) < 0.001:
            cycle_init_pos = cycle_init_pos + 1
            if cycle_init_pos >= max_cycle_init_pos:
                robot_init_pos = True
        
        if robot_init_pos:
            generate_hitting()

        # Update controller and get command
        controller.updateRobot(robot.joint_position,robot.joint_velocity,robot.joint_effort)
        command = controller.getCmd()

        # Send command
        send_command_agx(command)

        cycleCount = cycleCount + 1

