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

#　まる罰ゲームの実装

#path (x, y, pen_up_down, time)

rate = 100 #[hz]

path_buffer = []
path_index = 0


def create_path(x0, y0, x1, y1, t, n=100):
    path = []
    path.append((x0, y0, 0, 1.0))
    path.append((x0, y0, 1, 2.0))
    for i in range(n):
        x = x0 + (x1 - x0) * i / n
        y = y0 + (y1 - y0) * i / n
        path.append((x, y, 1, 2.0 + i * t / n))
    path.append((x1, y1, 0, 2.0 + t + 1.0))  
    path.append((x1, y1, 0, 2.0 + t + 2.0)) 
    return path

#丸罰ゲームの罫線の座標を返す
line_points = [
    [(100, 150), (250, 150)],
    [(100, 200), (250, 200)],
    [(150, 100), (150, 250)],
    [(200, 100), (200, 250)]
]

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

#円のpathを生成する関数
def target_circle(x0, y0, r, t, n=100):
    path = []
    path.append((x0, y0, 0, 1.0))
    path.append((x0, y0, 1, 2.0))
    for i in range(n):
        x = x0 + r * math.cos(2 * math.pi * i / n)
        y = y0 + r * math.sin(2 * math.pi * i / n)
        path.append((x, y, 1, 2.0 + i * t / n))
    path.append((x0 + r, y0, 0, 2.0 + t + 1.0))
    path.append((x0 + r, y0, 0, 2.0 + t + 2.0))
    return path

#バツのpathを生成する関数
def target_cross(x0, y0, r, t, n=100):
    points = [
        [(x0 - r, y0 - r), (x0 + r, y0 + r)],
        [(x0 - r, y0 + r), (x0 + r, y0 - r)]
    ]

    path = create_path(points[0][0][0], points[0][0][1], points[0][1][0], points[0][1][1], t / 2, n)
    path = path + create_path(points[1][0][0], points[1][0][1], points[1][1][0], points[1][1][1], t / 2, n)
    return path




def main():

    rospy.init_node('lesson_controller')
    publisher_angles = rospy.Publisher('joint_state', JointState, queue_size=10)
    #string型の⭕️と❌をトピックから受け取る
    # sub = rospy.Subscriber('circle_cross', String, callback)

    r = rospy.Rate(rate) #[hz] 
    theta1 = 0 #[rad]
    theta2 = 0  #[rad]
    t = 0 #[sec]

    arm = ControlArm(publisher_angles)

    # up pen
    #arm.up_pen()

    #罫線の描画 

    arm.up_pen()
    #1秒待つ
    rospy.sleep(1)

    def write(path):
        index = 0
        time = 0.0
        while index < len(path):
            x, y, z, target_time = path[index]
            if(z == 0):
                arm.up_pen()
            else:
                arm.down_pen()
            theta1, theta2 = arm.solve_ik_deg(x, y)
            arm.update_angles(theta1, theta2)
            print(x, y, z, target_time, theta1, theta2)
            time = time + 1 / rate
            if time >= target_time:
                index = index + 1
            r.sleep()

    for points in line_points:
        x0, y0 = points[0]
        # theta1, theta2 = arm.solve_ik_deg(x0, y0)
        # arm.update_angles(theta1, theta2)
        # rospy.sleep(1)
        # arm.down_pen()
        # rospy.sleep(1)
        x1, y1 = points[1]
        path = create_path(x0, y0, x1, y1, 2)
        # write(path)
        path_buffer.append(path)
    
    path = target_circle(175, 175, 20, 4)
    # write(path)
    path_buffer.append(path)

    path = target_cross(225, 175, 20, 6)
    # write(path)
    path_buffer.append(path)


    while not rospy.is_shutdown():

        # if len(path_buffer) > path_index:
        #     x, y = path_buffer[path_index]

        #     print(x, y)
        #     theta1, theta2 = arm.solve_ik_deg(x, y)

        #     arm.update_angles(theta1, theta2)

        #     path_index = path_index + 1
        #     if path_index >= len(path_buffer):
        #         path_index = 0
        #         path_buffer = []

        #     rospy.loginfo(x)
        if path_buffer:
            path = path_buffer.pop(0)
            write(path)
        r.sleep()


if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

