#!/usr/bin/env python3

import rospy
print("start")
import keyboard
print("keyboard inport")
from plotter_controller.srv import marubatu, marubatuRequest
print("inport srv")


print("start main")

rospy.init_node('key_input')
rospy.wait_for_service('marubatu_server')
marubatu_set = rospy.ServiceProxy('marubatu_server', marubatu)

check = False

while True:
    while True:
        check = not check
        if keyboard.is_pressed('q'):  # 'q'キーが押された場合
            print("qキーが押されました")
            xdata=0
            ydata=2
            break
        if keyboard.is_pressed('w'):  # 'w'キーが押された場合
            print("wキーが押されました")
            xdata=1
            ydata=2
            break
        if keyboard.is_pressed('e'):  # 'e'キーが押された場合
            print("eキーが押されました")
            xdata=2
            ydata=2
            break
        if keyboard.is_pressed('a'):  # 'a'キーが押された場合
            print("aキーが押されました")
            xdata=0
            ydata=1
            break
        if keyboard.is_pressed('s'):  # 's'キーが押された場合
            print("sキーが押されました")
            xdata=1
            ydata=1
            break
        if keyboard.is_pressed('d'):  # 'd'キーが押された場合
            print("dキーが押されました")
            xdata=2
            ydata=1
            break
        if keyboard.is_pressed('z'):  # 'z'キーが押された場合
            print("zキーが押されました")
            xdata=0
            ydata=0
            break
        if keyboard.is_pressed('x'):  # 'x'キーが押された場合
            print("xキーが押されました")
            xdata=1
            ydata=0
            break
        if keyboard.is_pressed('c'):  # 'c'キーが押された場合
            print("cキーが押されました")
            xdata=2
            ydata=0
            break

    data = marubatuRequest(marubatu=check,x=xdata,y=ydata)
    marubatu_set(data) 