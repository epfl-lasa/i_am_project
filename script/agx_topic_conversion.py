#!/usr/bin/env python3

import numpy as np
# import pinocchio as pin
import rospy

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates, LinkStates

class IiwaAgx:
    def __init__(self):
        self.q = [0, 0, 0, 0, 0, 0, 0]
        self.dq = [0, 0, 0, 0, 0, 0, 0]
        self.effort = [0, 0, 0, 0, 0, 0, 0]
        self.ee_pos = [0, 0, 0]

        self.object_pos_sub = rospy.Subscriber('/agx/object_pose', Pose, self.object_pose_callback, queue_size = 1)
        self.joint7_state_sub = rospy.Subscriber('/agx/joint7_state', Pose, self.joint7_state_callback, queue_size = 1)

        self.iiwa_joint1_angle = rospy.Subscriber('joint1_angle_output', Float64, self.joint_angle_pose_callback, 1, queue_size = 10)
        self.iiwa_joint1_velocity = rospy.Subscriber('joint1_velocity_output', Float64, self.joint_velocity_pose_callback, 1, queue_size = 10)
        self.iiwa_joint1_effort = rospy.Subscriber('joint1_force_output', Float64, self.joint_effort_pose_callback, 1, queue_size = 10)
        self.iiwa_joint2_angle = rospy.Subscriber('joint2_angle_output', Float64, self.joint_angle_pose_callback, 2, queue_size = 10)
        self.iiwa_joint2_velocity = rospy.Subscriber('joint2_velocity_output', Float64, self.joint_velocity_pose_callback, 2, queue_size = 10)
        self.iiwa_joint2_effort = rospy.Subscriber('joint2_force_output', Float64, self.joint_effort_pose_callback, 2, queue_size = 10)
        self.iiwa_joint3_angle = rospy.Subscriber('joint3_angle_output', Float64, self.joint_angle_pose_callback, 3,queue_size = 10)
        self.iiwa_joint3_velocity = rospy.Subscriber('joint3_velocity_output', Float64, self.joint_velocity_pose_callback, 3,queue_size = 10)
        self.iiwa_joint3_effort = rospy.Subscriber('joint3_force_output', Float64, self.joint_effort_pose_callback, 3,queue_size = 10)
        self.iiwa_joint4_angle = rospy.Subscriber('joint4_angle_output', Float64, self.joint_angle_pose_callback, 4, queue_size = 10)
        self.iiwa_joint4_velocity = rospy.Subscriber('joint4_velocity_output', Float64, self.joint_velocity_pose_callback, 4, queue_size = 10)
        self.iiwa_joint4_effort = rospy.Subscriber('joint4_force_output', Float64, self.joint_effort_pose_callback, 4, queue_size = 10)
        self.iiwa_joint5_angle = rospy.Subscriber('joint5_angle_output', Float64, self.joint_angle_pose_callback, 5, queue_size = 10)
        self.iiwa_joint5_velocity = rospy.Subscriber('joint5_velocity_output', Float64, self.joint_velocity_pose_callback, 5, queue_size = 10)
        self.iiwa_joint5_effort = rospy.Subscriber('joint5_force_output', Float64, self.joint_effort_pose_callback, 5, queue_size = 10)
        self.iiwa_joint6_angle = rospy.Subscriber('joint6_angle_output', Float64, self.joint_angle_pose_callback, 6, queue_size = 10)
        self.iiwa_joint6_velocity = rospy.Subscriber('joint6_velocity_output', Float64, self.joint_velocity_pose_callback, 6, queue_size = 10)
        self.iiwa_joint6_effort = rospy.Subscriber('joint6_force_output', Float64, self.joint_effort_pose_callback, 6, queue_size = 10)
        self.iiwa_joint7_angle = rospy.Subscriber('joint7_angle_output', Float64, self.joint_angle_pose_callback, 7, queue_size = 10)
        self.iiwa_joint7_velocity = rospy.Subscriber('joint7_velocity_output', Float64, self.joint_velocity_pose_callback, 7, queue_size = 10)
        self.iiwa_joint7_effort = rospy.Subscriber('joint7_force_output', Float64, self.joint_effort_pose_callback, 7, queue_size = 10)

        # Publisher
        self.object_pos_pub = rospy.Publisher('/gazebo/model_states', ModelStates, queue_size=1)
        self.joint7_state_pub = rospy.Publisher('/gazebo/link_states', LinkStates, queue_size=1)
        self.iiwa_joint_state_pub = rospy.Publisher('/iiwa/joint_states', JointState, queue_size=1)
 
    def object_pose_callback(self, msg):
        msg_pub = ModelStates()
        msg_pub.name = ['box_model']
        msg_pub.pose = [msg]

        self.object_pos_pub.publish(msg_pub)

    def joint7_state_callback(self, msg):
        msg_pub = LinkStates()
        msg_pub.name = ['iiwa::iiwa_link_7']
        msg_pub.pose = [msg]
        msg_pub.twist = [Twist()]

        self.joint7_state_pub.publish(msg_pub)

    def joint_angle_pose_callback(self, msg, arg):
        self.q[arg-1] = np.deg2rad(msg.data)

    def joint_velocity_pose_callback(self, msg, arg):
        self.dq[arg-1] = msg.data

    def joint_effort_pose_callback(self, msg, arg):
        self.effort[arg-1] = msg.data
        
    def publish_joint_state(self):
        msg_pub = JointState()
        msg_pub.position = self.q
        msg_pub.velocity = self.dq
        msg_pub.effort = self.effort
        self.iiwa_joint_state_pub.publish(msg_pub)

# def iiwa_joint7_pose_callback(msg):
#     msg_pub = LinkStates()
#     msg_pub.name = ['iiwa::iiwa_link_7']
#     msg_pub.pose = [msg]



if __name__ == '__main__':
    # Init ROS
    rospy.init_node("conversion_algoryx", anonymous=True)
    rate = rospy.Rate(100)

    communication = IiwaAgx()

    while not rospy.is_shutdown():
    
        communication.publish_joint_state()

        rate.sleep()
        # iiwa_joint7_pub = rospy.Publisher('/gazebo/link_states', LinkStates, queue_size=1)

        # rospy.spin()



# sudo python3 ../run-in-docker.py python3 click_application.py --model models/Robots/Panda/PandaSoftEE/PandaSoftEETorqueControl.yml --timeStep 0.005 --agxOnly --rcs --portRange 5656 5658



