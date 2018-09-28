#!/usr/bin/env python
import os
import copy
import math
import signal
import xmlrpclib
import pandas as pd
import os
import rospy
import roslib
from arl_hw_msgs.msg import Muscle, MuscleCommand, MusculatureState, MusculatureCommand


class Demonstration:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self._name = name
        self._ros_master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        self._musculature_command_publisher = rospy.Publisher('/musculature/command', MusculatureCommand, queue_size=10)

        self._rate_hz = 100
        self._steps_per_movement = self._rate_hz * 1
        self._idle = 0.0
        # read the csv file for specified pressures to generate trajectory
        cwd = os.path.abspath(os.path.join(__file__, "../.."))
        self._trajectory = pd.read_csv(cwd +'/config/'+'pressures_trajectory.csv')
        # reset all muscles in case of muscle memory
        self._initial = [0.0*i for i in range(32)]
        self._send_musculature_command(self._initial)
        self._muscles = self._trajectory.columns
        self._number_of_muscles = len(self._initial)
        self._execution_ended = False


    def run(self):
        rate = rospy.Rate(self._rate_hz)
        rospy.on_shutdown(self._shutdown_hook)

        while not rospy.is_shutdown():
            target = 0
            for i, pressures in self._trajectory.iterrows():
                if i+1 == len(self._trajectory):
                    break
                else:
                    initial_pressures = pressures.tolist()
                    target_pressures = self._trajectory.iloc[i+1].tolist()

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

                if self._execution_ended:
                    break

                target += 1

    def _shutdown_hook(self):
        self._execution_ended = True
        self._send_musculature_command(self._initial)

    def _send_musculature_command(self, muscles):
        musculature_command = MusculatureCommand()
        musculature_command.header.stamp = rospy.get_rostime()
        musculature_command.header.frame_id = '0'
        for num, muscle_press in enumerate(muscles):
            muscle_command = MuscleCommand()
            # when shut down go to reset
            if muscles == self._initial:
                muscle_command.name = 'muscle_' + str(num + 1)
            # else keep executing the specified trajectory
            else:
                muscle_command.name = self._muscles[num]
            muscle_command.pressure = muscle_press
            muscle_command.activation = 0
            muscle_command.control_mode = 0
            musculature_command.muscle_commands.append(muscle_command)
        self._musculature_command_publisher.publish(musculature_command)


if __name__ == '__main__':
    try:
        demo = Demonstration('robot_demo_node')
        demo.run()
    except rospy.ROSInterruptException:
        pass
