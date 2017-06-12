#!/usr/bin/env python
import os
import copy
import math
import xmlrpclib

import rospy
import roslib
from arl_hw_msgs.msg import Muscle, MuscleCommand, MusculatureState, MusculatureCommand

class Flexer:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self._name = name
        self._ros_master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        self._musculature_command_publisher = rospy.Publisher('/muscle_muxer/musculature_command', MusculatureCommand, queue_size=10)
        #self._selected_muscle_names = ['muscle_1','muscle_2','muscle_5','muscle_8','muscle_11','muscle_15','muscle_16','muscle_23','muscle_24','muscle_26']
        self._selected_muscle_names = ['muscle_24','muscle_27', 'muscle_25', 'muscle_26']
        self._number_of_muscles = len(self._selected_muscle_names)
        self._rate_hz = 100
        self._min_pressure = 4500.0
        self._max_pressure = 7500.0
        self._step_size = (1.0 / self._rate_hz) * math.pi

    def run(self):
        rate = rospy.Rate(self._rate_hz)
        step = 0
        while not rospy.is_shutdown():
            step = (step + 1) % self._rate_hz
            self._send_musculature_command(step)
            rate.sleep()

    def _send_musculature_command(self, step):
        offset = math.pi / self._number_of_muscles
        musculature_command = MusculatureCommand()
        musculature_command.header.stamp = rospy.get_rostime()
        musculature_command.header.frame_id = '0'
        for num, muscle in enumerate(self._selected_muscle_names):
            off_step = ((self._step_size * step) + (offset * num)) % math.pi
            pressure = (math.sin(off_step) * (self._max_pressure - self._min_pressure)) + self._min_pressure
            muscle_command = MuscleCommand()
            muscle_command.name = self._selected_muscle_names[num]
            muscle_command.pressure = pressure
            muscle_command.activation = 0
            muscle_command.control_mode = 0
            musculature_command.muscle_commands.append(muscle_command)
        self._musculature_command_publisher.publish(musculature_command)

if __name__ == '__main__':
    try:
        flexer = Flexer('muscle_flexer_node')
        flexer.run()
    except rospy.ROSInterruptException:
        pass
