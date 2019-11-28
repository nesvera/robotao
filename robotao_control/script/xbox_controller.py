#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

vel_cmd = Twist()

def joy_callback(data):
    global vel_cmd
    vel_cmd = Twist()

    axes = data.axes
    buttons = data.buttons

    # linear commands
    left_stick_x = axes[0]      # 1 = left, -1 = right
    left_stick_y = axes[1]      # 1 = front, -1 = back

    # angular commands
    right_stick_x = axes[3]     # 1 = left, -1 = right

    # jump command
    btn_a = buttons[0]

    vel_cmd.linear.x = left_stick_x
    vel_cmd.linear.y = left_stick_y
    vel_cmd.linear.z = btn_a
    vel_cmd.angular.z = right_stick_x

if __name__ == "__main__":

    rospy.init_node("xbox_controller")

    rospy.Subscriber("/joy", Joy, joy_callback)
    vel_pub = rospy.Publisher("/robotao/vel_cmd", Twist, queue_size=1)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        print(vel_cmd)
        
        vel_pub.publish(vel_cmd)
        rate.sleep()