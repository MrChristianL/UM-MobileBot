#! /usr/bin/python

import lcm
from time import sleep
import sys
sys.path.append("lcmtypes")

from lcmtypes import mbot_encoder_t
from lcmtypes import mbot_imu_t
from lcmtypes import mbot_motor_command_t
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
from lcmtypes import timestamp_t

sq_w = 1.0
sq_d = 1.0

class WaypointFollower():
    def __init__(self):
        """ Setup LCM and subscribe """
        lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.lcm_sub = self.lc.subscribe("ODOMETRY", self.odometry_handler)
        self.waypoints = [[0.0,0.0],[sq_w,0.0],[sq_w,sq_d],[0.0,sq_d],[0.0,0.0]]
        self.wpt_num = 0
        self.wpt_thresh = 0.01

    def odometry_handler(self, channel, data):
        msg = odometry_t().decode(data)
        msg.x
        msg.y
        msg.theta

    def motor_cmd_publish():
        stop_msg = mbot_motor_command_t()
        msg.utime = 0.0
        msg.trans_v = 0.0
        msg.angular_v = 0.0

        drive_61_msg = mbot_motor_command_t()
        msg.utime = 0.0
        msg.trans_v = 0.61
        msg.angular_v = 0.0

        drive_122_msg = mbot_motor_command_t()
        msg.utime = 0.0
        msg.trans_v = 1.22
        msg.angular_v = 0.0

        right_turn_msg = mbot_motor_command_t()
        msg.utime = 0.0
        msg.trans_v = 0.0
        msg.angular_v = 3.1415/4.0

        left_turn_msg = mbot_motor_command_t()
        msg.utime = 0.0
        msg.trans_v = 0.0
        msg.angular_v = -3.1415/4.0

        # Staright 61cm
        self.lc.publish("MBOT_MOTOR_COMMAND",drive_61_msg.encode())
        sleep(3.05) # 0.2m/s
        # Turn right
        self.lc.publish("MBOT_MOTOR_COMMAND",right_turn_msg.encode())
        sleep(2.0)
        # Staright 61cm
        self.lc.publish("MBOT_MOTOR_COMMAND",drive_61_msg.encode())
        sleep(3.05) # 0.2m/s
        # Turn Left
        self.lc.publish("MBOT_MOTOR_COMMAND",left_turn_msg.encode())
        sleep(2.0)
        # Staright 61cm
        self.lc.publish("MBOT_MOTOR_COMMAND",drive_61_msg.encode())
        sleep(3.05) # 0.2m/s
        # Turn Left
        self.lc.publish("MBOT_MOTOR_COMMAND",left_turn_msg.encode())
        sleep(2.0)
        # Straight 122cm
        self.lc.publish("MBOT_MOTOR_COMMAND",drive_122_msg.encode())
        sleep(3.05) # 0.2m/s
        # Turn Right
        self.lc.publish("MBOT_MOTOR_COMMAND",right_turn_msg.encode())
        sleep(2.0)
        # Straight 61cm
        self.lc.publish("MBOT_MOTOR_COMMAND",drive_61_msg.encode())
        sleep(3.05) # 0.2m/s
        # Turn Right
        self.lc.publish("MBOT_MOTOR_COMMAND",right_turn_msg.encode())
        sleep(2.0)
        # Straight 122cm
        self.lc.publish("MBOT_MOTOR_COMMAND",drive_122_msg.encode())
        sleep(3.05) # 0.2m/s
        # Turn Left
        self.lc.publish("MBOT_MOTOR_COMMAND",left_turn_msg.encode())
        sleep(2.0)
        # Straight 61cm
        self.lc.publish("MBOT_MOTOR_COMMAND",drive_61_msg.encode())
        sleep(3.05) # 0.2m/s
        # Turn Left
        self.lc.publish("MBOT_MOTOR_COMMAND",left_turn_msg.encode())
        sleep(2.0)
        # Straight 61cm
        self.lc.publish("MBOT_MOTOR_COMMAND",drive_61_msg.encode())
        sleep(3.05) # 0.2m/s
        # Turn Right
        self.lc.publish("MBOT_MOTOR_COMMAND",right_turn_msg.encode())
        sleep(2.0)
        # Straight 61cm
        self.lc.publish("MBOT_MOTOR_COMMAND",drive_61_msg.encode())
        sleep(3.05) # 0.2m/s
        # Stop
        self.lc.publish("MBOT_MOTOR_COMMAND",stop_msg.encode())




