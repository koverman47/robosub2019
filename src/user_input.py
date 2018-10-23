#!/usr/bin/env python

import rospy
import math

from robosub2019.msg import Key
from robosub2019.msg import MotorCommands


'''
MotorCommands:
    0: Port Forward
    1: Starboard Forward
    2: Fore Strafe
    3: Aft Strafe
    4: Port Fore Depth
    5: Starboard Fore Depth
    6: Port Aft Depth
    7: Starboard Aft Depth
'''


## Start Variables
com_pub = None

com_msg = None

command_timeout = dict()
timeout_delay = 200

power = 1

key_mappings = {119:    {0: 1, 1: 1},   # w - forward
                97:     {2: -1, 3: -1}, # a - strafe left
                115:    {0: -1, 1: -1}, # s - reverse
                100:    {2: 1, 3: 1},   # d - strafe right
                113:    {2: -1, 3: 1},  # q - rotate left
                101:    {2: 1, 3: -1},  # e - rotate right
                99:     {4: -1, 5: -1, 6: -1, 7: -1}, # c - descend
                32:     {4: 1, 5: 1, 6: 1, 7: 1}, # space - ascend
                120:    {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0}, # x - stop all
                116:    {0: 1, 1: 1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 1, 7: 1}, # t - start all
                49:     {0: 1}, # 1 - port forward
                50:     {1: 1}, # 2 - starboard forward
                51:     {2: 1}, # 3 - fore strafe
                52:     {3: 1}, # 4 - aft strafe
                53:     {4: 1}, # 5 - port fore depth
                54:     {5: 1}, # 6 - starboard fore depth
                55:     {6: 1}, # 7 - port aft depth
                56:     {7: 1}} # 7 - starboard aft depth

## End Variables


def user_input(key, down):
    global command_timeout
    
    key = int(key.code)

    if not down:
        command_timeout[key] = datetime.now().microsecond + timeout_delay
        return

    set_commands(key)


def set_commands(key, power = 1):
    global com_msg

    if key in key_mappings:
        for key, value in key_mappings[key].items():
            com_msg.commands[key] = value * power


def key_down(key):
    user_input(key, True)


def key_up(key):
    user_input(key, False)


def send_commands():
    global com_msg, com_pub

    com_msg.header.seq += 1
    com_msg.header.stamp = rospy.get_rostime()
    
    com_pub.publish(com_msg)



def main():
    global com_msg, com_pub, command_timeout

    rospy.init_node('UserInput')
    rospy.Subscriber("keyboard/keydown", Key, key_down)
    rospy.Subscriber("keyboard/keyup", Key, key_up)

    com_pub = rospy.Publisher("command/motor", MotorCommands, queue_size=32)

    com_msg = MotorCommands()
    com_msg.header.seq = 0
    com_msg.header.stamp = rospy.get_rostime()
    com_msg.header.frame_id = "0"

    rate = rospy.Rate(8)
    while not rospy.is_shutdown():
        # Need timeout because of remote desktop
        for key, value in command_timeout.items():
            if datetime.now().microsecond > value:
                set_commands(key, 0)
                command_timeout.pop(key, None)

        send_commands()
        rate.sleep()
    
    rospy.spin()


if __name__ == "__main__":
    main()


