#!/usr/bin/env python3

import rospy
print("start")
import readchar

print("keyboard inport")
from plotter_controller.srv import marubatu, marubatuRequest
print("inport srv")


print("start main")

rospy.init_node('key_input')
rospy.wait_for_service('marubatu_server')
marubatu_set = rospy.ServiceProxy('marubatu_server', marubatu)

check = False

xdata = 0
ydata = 0

while True:
    check = not check
    while True:
        c = readchar.readchar()
        if c == 'q':  # 'q'キーが押された場合
            print("qキーが押されました")
            xdata=0
            ydata=2
            break
        if c == 'w':  # 'w'キーが押された場合
            print("wキーが押されました")
            xdata=1
            ydata=2
            break
        if c == 'e':  # 'e'キーが押された場合
            print("eキーが押されました")
            xdata=2
            ydata=2
            break
        if c == 'a':  # 'a'キーが押された場合
            print("aキーが押されました")
            xdata=0
            ydata=1
            break
        if c == 's':  # 's'キーが押された場合
            print("sキーが押されました")
            xdata=1
            ydata=1
            break
        if c == 'd':  # 'd'キーが押された場合
            print("dキーが押されました")
            xdata=2
            ydata=1
            break
        if c == 'z':  # 'z'キーが押された場合
            print("zキーが押されました")
            xdata=0
            ydata=0
            break
        if c == 'x':  # 'x'キーが押された場合
            print("xキーが押されました")
            xdata=1
            ydata=0
            break
        if c == 'c':  # 'c'キーが押された場合
            print("cキーが押されました")
            xdata=2
            ydata=0
            break

    print("push key")
    data = marubatuRequest(marubatu=check,x=xdata,y=ydata)
    print(data)
    marubatu_set(data) 