#!/usr/bin/env python3

import numpy as np
import rospy
# from mc_rbdyn_urdf import URDFParserResult
from iiwa_tools.srv import GetGravity


from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState

from enum import Enum

Control = Enum('Control', ['ANGLE', 'TORQUE'])

CONTROL_AGX = Control.TORQUE


class BridgeGazeboAgx:
    def __init__(self, control_type):
        self.q = np.array([0,0,0,0,0,0,0])
        self.dq = np.array([0,0,0,0,0,0,0])
        self.effort = np.array([0,0,0,0,0,0,0])

        if  control_type == Control.ANGLE:
            joint_state = rospy.Subscriber('/iiwa/joint_states', JointState, self.send_command_callback, queue_size = 1)

            # AGX publisher
            self.joint1_pub = rospy.Publisher('/iiwa_joint_1', Float64, queue_size=1)
            self.joint2_pub = rospy.Publisher('/iiwa_joint_2', Float64, queue_size=1)
            self.joint3_pub = rospy.Publisher('/iiwa_joint_3', Float64, queue_size=1)
            self.joint4_pub = rospy.Publisher('/iiwa_joint_4', Float64, queue_size=1)
            self.joint5_pub = rospy.Publisher('/iiwa_joint_5', Float64, queue_size=1)
            self.joint6_pub = rospy.Publisher('/iiwa_joint_6', Float64, queue_size=1)
            self.joint7_pub = rospy.Publisher('/iiwa_joint_7', Float64, queue_size=1)

        elif control_type == Control.TORQUE:
            torque_quat_sub = rospy.Subscriber('/iiwa/TorqueController/command', Float64MultiArray, self.send_command_torque_callback, queue_size = 1)

            # AGX publisher
            self.joint1_pub = rospy.Publisher('/iiwa_joint_1', Float64, queue_size=1)
            self.joint2_pub = rospy.Publisher('/iiwa_joint_2', Float64, queue_size=1)
            self.joint3_pub = rospy.Publisher('/iiwa_joint_3', Float64, queue_size=1)
            self.joint4_pub = rospy.Publisher('/iiwa_joint_4', Float64, queue_size=1)
            self.joint5_pub = rospy.Publisher('/iiwa_joint_5', Float64, queue_size=1)
            self.joint6_pub = rospy.Publisher('/iiwa_joint_6', Float64, queue_size=1)
            self.joint7_pub = rospy.Publisher('/iiwa_joint_7', Float64, queue_size=1)

    def send_command_callback(self, command):
        self.joint1_pub.publish(np.rad2deg(command.position[0]))
        self.joint2_pub.publish(np.rad2deg(command.position[1]))
        self.joint3_pub.publish(np.rad2deg(command.position[2]))
        self.joint4_pub.publish(np.rad2deg(command.position[3]))
        self.joint5_pub.publish(np.rad2deg(command.position[4]))
        self.joint6_pub.publish(np.rad2deg(command.position[5]))
        self.joint7_pub.publish(np.rad2deg(command.position[6]))

    def send_command_torque_callback(self, torque):
        self.joint1_pub.publish(torque.data[0]) #/100)
        self.joint2_pub.publish(torque.data[1]) #/100)
        self.joint3_pub.publish(torque.data[2]) #/100)
        self.joint4_pub.publish(torque.data[3]) #/100)
        self.joint5_pub.publish(torque.data[4]) #/100)
        self.joint6_pub.publish(torque.data[5]) #/100)
        self.joint7_pub.publish(torque.data[6]) #/100)

if __name__ == '__main__':
    # Init ROS
    rospy.init_node("control_algoryx", anonymous=True)
    rate = rospy.Rate(200)
    control_agx = BridgeGazeboAgx(CONTROL_AGX)

    while not rospy.is_shutdown():
    
        # communication.publish_joint_state()

        rate.sleep()


    # Init subscriber and publisher

    # if  CONTROL_AGX == Control.ANGLE:
    #     joint_state = rospy.Subscriber('/iiwa/joint_states', JointState, send_command_callback, queue_size = 1)

    #     # AGX publisher
    #     joint1_pub = rospy.Publisher('/iiwa_joint_1', Float64, queue_size=1)
    #     joint2_pub = rospy.Publisher('/iiwa_joint_2', Float64, queue_size=1)
    #     joint3_pub = rospy.Publisher('/iiwa_joint_3', Float64, queue_size=1)
    #     joint4_pub = rospy.Publisher('/iiwa_joint_4', Float64, queue_size=1)
    #     joint5_pub = rospy.Publisher('/iiwa_joint_5', Float64, queue_size=1)
    #     joint6_pub = rospy.Publisher('/iiwa_joint_6', Float64, queue_size=1)
    #     joint7_pub = rospy.Publisher('/iiwa_joint_7', Float64, queue_size=1)

    # elif CONTROL_AGX == Control.TORQUE:

    #     torque_quat_sub = rospy.Subscriber('/iiwa/TorqueController/command', Float64MultiArray, send_command_torque_callback, queue_size = 1)

    #     # AGX publisher
    #     joint1_pub = rospy.Publisher('/iiwa_joint_1', Float64, queue_size=1)
    #     joint2_pub = rospy.Publisher('/iiwa_joint_2', Float64, queue_size=1)
    #     joint3_pub = rospy.Publisher('/iiwa_joint_3', Float64, queue_size=1)
    #     joint4_pub = rospy.Publisher('/iiwa_joint_4', Float64, queue_size=1)
    #     joint5_pub = rospy.Publisher('/iiwa_joint_5', Float64, queue_size=1)
    #     joint6_pub = rospy.Publisher('/iiwa_joint_6', Float64, queue_size=1)
    #     joint7_pub = rospy.Publisher('/iiwa_joint_7', Float64, queue_size=1)

    # rospy.spin()



# sudo python3 ../run-in-docker.py python3 click_application.py --model models/Robots/Panda/PandaSoftEE/PandaSoftEETorqueControl.yml --timeStep 0.005 --agxOnly --rcs --portRange 5656 5658



