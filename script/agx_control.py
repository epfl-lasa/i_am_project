#!/usr/bin/env python3

import numpy as np
import rospy


from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

def command_callback(command):
    joint1_pub.publish(np.rad2deg(command.position[0]))
    joint2_pub.publish(np.rad2deg(command.position[1]))
    joint3_pub.publish(np.rad2deg(command.position[2]))
    joint4_pub.publish(np.rad2deg(command.position[3]))
    joint5_pub.publish(np.rad2deg(command.position[4]))
    joint6_pub.publish(np.rad2deg(command.position[5]))
    joint7_pub.publish(np.rad2deg(command.position[6]))
    

if __name__ == '__main__':
    # Init ROS
    rospy.init_node("control_algoryx", anonymous=True)
    rate = rospy.Rate(100)
    # Init subscriber and publisher
    command_torque_sub = rospy.Subscriber('/iiwa/joint_states', JointState, command_callback, queue_size = 1)


    joint1_pub = rospy.Publisher('/iiwa_joint_1', Float64, queue_size=1)
    joint2_pub = rospy.Publisher('/iiwa_joint_2', Float64, queue_size=1)
    joint3_pub = rospy.Publisher('/iiwa_joint_3', Float64, queue_size=1)
    joint4_pub = rospy.Publisher('/iiwa_joint_4', Float64, queue_size=1)
    joint5_pub = rospy.Publisher('/iiwa_joint_5', Float64, queue_size=1)
    joint6_pub = rospy.Publisher('/iiwa_joint_6', Float64, queue_size=1)
    joint7_pub = rospy.Publisher('/iiwa_joint_7', Float64, queue_size=1)

    # while not rospy.is_shutdown():

    #     rate.sleep()


    rospy.spin()



# sudo python3 ../run-in-docker.py python3 click_application.py --model models/Robots/Panda/PandaSoftEE/PandaSoftEETorqueControl.yml --timeStep 0.005 --agxOnly --rcs --portRange 5656 5658



