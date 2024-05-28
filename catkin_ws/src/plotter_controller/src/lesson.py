#!/usr/bin/env python3

import math
import numpy as np

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import time
from sensor_msgs.msg import JointState

from control_arm import ControlArm


def target_line(t):

    x = 50*np.sin((2*np.pi/5)*t) + 150
    y = 50*np.cos((2*np.pi/5)*t) + 150

    return x, y


def main():

    rospy.init_node('lesson_controller')
    publisher_angles = rospy.Publisher('joint_state', JointState, queue_size=10)

    rate = 100 #[hz]

    r = rospy.Rate(rate) #[hz] 
    theta1 = 0 #[rad]
    theta2 = 0  #[rad]
    t = 0 #[sec]

    arm = ControlArm(publisher_angles)

    # up pen
    #arm.up_pen()

    while not rospy.is_shutdown():

        x, y = target_line(t)
        print(x, y)
        theta1, theta2 = arm.solve_ik_deg(x, y)

        #theta1 = 0.0
        #theta2 = 0.0
        arm.update_angles(theta1, theta2)

        t = t + (1/rate) #[sec]

        rospy.loginfo(x)
        r.sleep()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

