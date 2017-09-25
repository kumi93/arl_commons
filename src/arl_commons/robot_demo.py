#!/usr/bin/env python
import os
import copy
import math
import signal
import xmlrpclib

import rospy
import roslib
from arl_hw_msgs.msg import Muscle, MuscleCommand, MusculatureState, MusculatureCommand


class Demonstration:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self._name = name
        self._ros_master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        self._musculature_command_publisher = rospy.Publisher('/muscle_muxer/musculature_command', MusculatureCommand, queue_size=10)
        self._musculature_state_subscriber = rospy.Subscriber('/muscle_muxer/musculature_state', MusculatureState, self._musculature_state_callback, queue_size=10)

        self._rate_hz = 10
        self._steps_per_movement = self._rate_hz * 10
        self._idle = 4000.0
        self._initial = [self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle, self._idle]
        self._position_1 = [self._idle, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, 8000, self._idle, 8000, self._idle, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 8000]
        self._position_2 = [self._idle, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 8000, self._idle, 10000, 4000]
        self._position_3 = [self._idle, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, 6500, 8000]
        self._position_4 = [self._idle, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, 10000, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, 10000, 4000]
        self._position_5 = [self._idle, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, 6000, self._idle, 8000, self._idle, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, 8000, self._idle]
        self._position_6 = [self._idle, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, 10000, self._idle, 8000, self._idle, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, 8000, self._idle]
        self._position_7 = [self._idle, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, 10000, self._idle, 8000, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, 8000, 6500]
        self._position_8 = [self._idle, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, 4000, 10000]
        self._position_9 = [self._idle, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, self._idle, self._idle, 8000, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, 8000, 6500]
        self._position_10 = [self._idle, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, 10000, 10000, self._idle, self._idle, 10000, 10000, self._idle, self._idle, 4000, 10000, 10000, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, 4000, 10000]
        self._position_11 = [self._idle, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, 10000, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 4000, 10000, 10000, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, 4000, 10000]
        self._position_12 = [self._idle, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, 10000, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, 4000, 10000]
        self._position_13 = [self._idle, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, 10000, self._idle, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, 10000, 5000]
        self._position_14 = [self._idle, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, 6000, self._idle, 8000, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, 8000, 6500]
        self._position_15 = [self._idle, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, 6000, self._idle, 8000, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, 10000]
        self._position_16 = [self._idle, self._idle, self._idle, 10000, 10000, self._idle, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, self._idle, self._idle, 6000, self._idle, 8000, self._idle, self._idle, 10000, self._idle, self._idle, 10000, self._idle, 10000, self._idle]

        self._target_positions = [self._initial, self._position_1, self._position_2, self._position_3, self._position_4, self._position_5, self._position_6, self._position_7, self._position_8,
                           self._position_9, self._position_10, self._position_11, self._position_12, self._position_13, self._position_14, self._position_15, self._position_16]
        self._current_pressures = []
        self._number_of_muscles = len(self._initial)
        self._execution_ended = False

    def run(self):
        rate = rospy.Rate(self._rate_hz)
        rospy.on_shutdown(self._shutdown_hook)

        while len(self._current_pressures) == 0 and not rospy.is_shutdown():
            rate.sleep()

        while not rospy.is_shutdown():
            target = 0
            for target_pressures in self._target_positions:
                initial_pressures = copy.deepcopy(self._current_pressures)

                print 'Moving to target ' + str(target)

                for t_idx in range(0, self._steps_per_movement):
                    next_pressures = []
                    for num, target_pressure in enumerate(target_pressures):
                        t_slice = (target_pressure - initial_pressures[num]) / self._steps_per_movement
                        next_pressure = (t_slice * t_idx) + initial_pressures[num]
                        next_pressures.append(next_pressure)

                    if self._execution_ended:
                        break
                    else:
                        self._send_musculature_command(next_pressures)
                        rate.sleep()

                target += 1

    def _shutdown_hook(self):
        self._execution_ended = True
        self._musculature_state_subscriber.unregister()
        self._send_musculature_command(self._initial)

    def _send_musculature_command(self, muscles):
        musculature_command = MusculatureCommand()
        musculature_command.header.stamp = rospy.get_rostime()
        musculature_command.header.frame_id = '0'
        for num, muscle_press in enumerate(muscles):
            muscle_command = MuscleCommand()
            muscle_command.name = 'muscle_' + str(num + 1)
            muscle_command.pressure = muscle_press
            muscle_command.activation = 0
            muscle_command.control_mode = 0
            musculature_command.muscle_commands.append(muscle_command)
        self._musculature_command_publisher.publish(musculature_command)

    def _musculature_state_callback(self, msg):
        pressures = []
        for muscle_state in msg.muscle_states:
            pressures.append(muscle_state.current_pressure)
        self._current_pressures = pressures


if __name__ == '__main__':
    try:
        demo = Demonstration('robot_demo_node')
        demo.run()
    except rospy.ROSInterruptException:
        pass
