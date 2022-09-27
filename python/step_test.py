#! /usr/bin/python
import lcm
from time import sleep
import sys
sys.path.append("lcmtypes")

from lcmtypes import mbot_motor_command_t

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

stop_command = mbot_motor_command_t()
stop_command.trans_v = 0.0
stop_command.angular_v = 0.0

#drive_command1 = mbot_motor_command_t()
#drive_command1.trans_v = 0.00
#drive_command1.angular_v = 0.25

drive_command2 = mbot_motor_command_t()
drive_command2.trans_v = 0.25
drive_command2.angular_v = 0.00

#lc.publish("MBOT_MOTOR_COMMAND",drive_command1.encode())
#sleep(5.0)
lc.publish("MBOT_MOTOR_COMMAND",drive_command2.encode())
sleep(8.0)
lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
sleep(1.0)