#! /usr/bin/python
import lcm
import time
import sys
sys.path.append("lcmtypes")

# from lcmtypes import mbot_encoder_t
# from lcmtypes import mbot_imu_t
# from lcmtypes import mbot_motor_command_t
# from lcmtypes import odometry_t
# from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
#from lcmtypes import timestamp_t
from lcmtypes import message_received_t


def my_handler(channel, data):
    msg = message_received_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   time of confirmation creation   = %s" % str(msg.utime))
    print("   time of message creation    = %s" % str(msg.creation_time))
    print("   name of channel = %s" % str(msg.channel))
    print("")

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

full_path = robot_path_t()
full_path.path_length = 2
full_path.utime = time.now()

full_path.path[0].x = 1
full_path.path[0].y = 0
full_path.path[0].theta = 90

full_path.path[1].x = 1
full_path.path[1].y = 1
full_path.path[1].theta = 90

lc.publish("CONTROLLER_PATH_CHANNEL",full_path.encode())

validation_msg = lc.subscribe("MESSAGE_CONFIRMATION_CHANNEL",my_handler)