#!/usr/bin/env python
import os
import copy
import math
import xmlrpclib

import rospy
import roslib
from arl_hw_msgs.msg import Muscle, MuscleCommand, MusculatureState, MusculatureCommand


class MuscleReflex:
    def __init__(self):
        self._current_pressure = 0.0
        self._last_pressure_set = 6500.0
        self._goal_pressure = 6500.0
        self._amplitude = 50.0

    def process(self, muscle_state):
        self._current_pressure = muscle_state.current_pressure

        diff = self._last_pressure_set - self._current_pressure
        if math.fabs(diff) > self._amplitude:
            print muscle_state.name + ' off by: ' + str(diff)
            self._goal_pressure = self._last_pressure_set + diff
            self._amplitude = math.fabs(diff)

        if math.fabs(self._goal_pressure - self._current_pressure) < 20:
            self._last_pressure_set = self._goal_pressure
            self._amplitude = 50.0

        return self._goal_pressure

class Follower:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self._name = name
        self._ros_master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        self._musculature_command_publisher = rospy.Publisher('/muscle_muxer/musculature_command', MusculatureCommand, queue_size=10)
        rospy.Subscriber('/muscle_muxer/musculature_state', MusculatureState, self._muscle_state_callback)
        #self._selected_muscle_names = ['muscle_1','muscle_2','muscle_5','muscle_8','muscle_11','muscle_15','muscle_16','muscle_23','muscle_24','muscle_26']
        self._selected_muscle_names = ['muscle_24', 'muscle_25']
        self._number_of_muscles = len(self._selected_muscle_names)
        self._rate_hz = 100
        self._callback_data_in_use = False
        self._musculatur_state = MusculatureState()
        self._musculatur_state_cache = MusculatureState()
        self._musculatur_state_cache_used = False

        self._muscle_reflexes = {}
        for muscle_name in self._selected_muscle_names:
            self._muscle_reflexes[muscle_name] = MuscleReflex()


    def run(self):
        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown():
            self._callback_data_in_use = True
            commands = {}
            for muscle in self._musculatur_state.muscle_states:
                if muscle.name in self._selected_muscle_names:
                    commands[muscle.name] = self._muscle_reflexes[muscle.name].process(muscle)
            self._send_musculature_command(commands)
            self._update_musculature_missed_state()
            rate.sleep()

    def _send_musculature_command(self, commands):
        musculature_command = MusculatureCommand()
        musculature_command.header.stamp = rospy.get_rostime()
        musculature_command.header.frame_id = '0'
        for muscle_name, pressure in commands.iteritems():
            muscle_command = MuscleCommand()
            muscle_command.name = muscle_name
            muscle_command.pressure = pressure
            muscle_command.activation = 0
            muscle_command.control_mode = 0
            musculature_command.muscle_commands.append(muscle_command)
        self._musculature_command_publisher.publish(musculature_command)

    def _update_musculature_missed_state(self):
        if self._musculatur_state_cache_used:
            self._musculatur_state = self._musculatur_state_cache
            self._musculatur_state_cache_used = False

    def _muscle_state_callback(self, data):
        if not self._callback_data_in_use:
            self._musculature_state = data
        else:
            self._musculatur_state_cache = data
            self._musculatur_state_cache_used = True

if __name__ == '__main__':
    try:
        follower = Follower('follower_node')
        follower.run()
    except rospy.ROSInterruptException:
        pass
