#!/usr/bin/env python
import os
import copy
import math
import signal
import xmlrpclib
from enum import Enum
from time import sleep

import rospy
import roslib
from arl_hw_msgs.msg import Muscle, MuscleCommand, MusculatureState, MusculatureCommand


class ArlArmException(Exception):
    def __init__(self, msg):
        self._msg = msg

    @property
    def msg(self):
        return self._msg


class ArlMuscleControlMode(Enum):
    BY_PRESSURE = 0
    BY_ACTIVATION = 1


class ArlMusculature:
    def __init__(self):
        self._muscle_states = {}
        self._muscle_commands = {}

    @property
    def muscle_states(self):
        return self._muscle_states

    @muscle_states.setter
    def muscle_states(self, value):
        self._muscle_states = value

    @property
    def muscle_commands(self):
        return self._muscle_commands

    @muscle_commands.setter
    def muscle_commands(self, value):
        self._muscle_commands = value

    def set_muscle_command(self, command):
        self._muscle_commands[command.name] = command

    def set_muscle_state(self, state):
        self._muscle_states[state.name] = state

    def get_muscle_command(self, muscle_name):
        return self._muscle_commands[muscle_name]

    def get_muscle_state(self, muscle_name):
        return self._muscle_states[muscle_name]


class ArlMuscleCommand:
    def __init__(self, name='', activation=0.0, pressure=0.0, control_mode=ArlMuscleControlMode.BY_ACTIVATION):
        self._name = name
        self._activation = activation
        self._pressure = pressure
        self._control_mode = control_mode

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self.name = value

    @property
    def activation(self):
        return self._activation

    @activation.setter
    def activation(self, value):
        self._activation = value

    @property
    def pressure(self):
        return self._pressure

    @pressure.setter
    def pressure(self, value):
        self._pressure = value

    @property
    def control_mode(self):
        self._control_mode

    @control_mode.setter
    def control_mode(self, value):
        self.control_mode = value


class ArlMuscleState:
    def __init__(self, name='', current_pressure=0.0, desired_pressure=0.0, activation=0.0, tension=0.0, tension_filtered=0.0, control_mode=ArlMuscleControlMode.BY_ACTIVATION):
        self._name = name
        self._current_pressure = current_pressure
        self._desired_pressure = desired_pressure
        self._activation = activation
        self._tension = tension
        self._tension_filtered = tension_filtered
        self._control_mode = control_mode

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, name):
        self._name = name

    @property
    def current_pressure(self):
        return self.current_pressure

    @current_pressure.setter
    def current_pressure(self, value):
        self.current_pressure = value

    @property
    def desired_pressure(self):
        return self._desired_pressure

    @desired_pressure.setter
    def desired_pressure(self, value):
        self._desired_pressure = value

    @property
    def activation(self):
        return self._activation

    @activation.setter
    def activation(self, value):
        self._activation = value

    @property
    def tension(self):
        return self._tension

    @tension.setter
    def tension(self, value):
        self._tension = value

    @property
    def tension_filtered(self):
        return self._tension_filtered

    @tension_filtered.setter
    def tension_filtered(self, value):
        self._tension_filtered = value

    @property
    def control_mode(self):
        return self.control_mode

    @control_mode.setter
    def control_mode(self, value):
        self.control_mode = value


class ArlArm:
    def __init__(self):
        rospy.init_node('pyarlarm_node', anonymous=True)
        self._ros_master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        self._musculature_command_publisher = rospy.Publisher('/muscle_muxer/musculature_command', MusculatureCommand, queue_size=10)
        rospy.Subscriber('/muscle_muxer/musculature_state', MusculatureState, self._musculature_state_callback)
        self._musculature_state = ArlMusculature()

        signal.signal(signal.SIGINT, self.sig_handler)
        signal.signal(signal.SIGTERM, self.sig_handler)
        self._ok = True

        #waiting for publishers and subscribers to register completely
        sleep(0.3)

    @property
    def ok(self):
        return self._ok

    def sig_handler(self,signum, frame):
        self._ok = False

    def shutdown(self):
        rospy.signal_shutdown('shutdown by user')

    def _musculature_state_callback(self, msg):
        for muscle_state in msg.muscle_states:
            muscle = ArlMuscleState()
            muscle.name = muscle_state.name
            muscle.current_pressure = muscle_state.current_pressure
            muscle.desired_pressure = muscle_state.desired_pressure
            muscle.activation = muscle_state.activation
            muscle.tension = muscle_state.tension
            muscle.tension_filtered = muscle_state.tension_filtered
            if muscle_state.control_mode == MuscleCommand.CONTROL_MODE_BY_ACTIVATION:
                muscle.control_mode = ArlMuscleControlMode.BY_ACTIVATION
            else:
                muscle.control_mode = ArlMuscleControlMode.BY_PRESSURE
            self._musculature_state.set_muscle_state(muscle)

    def receiveMusculatureState(self):
        try:
            return copy.deepcopy(self._musculature_state)
        except:
            raise ArlArmException('Error while retrieving the musculature')


    def receiveMuscleState(self, muscle_name):
        try:
            return copy.deepcopy(self._musculature_state.get_muscle_state(muscle_name))
        except:
            raise ArlArmException('Error while retrieving specific muscle')

    def sendMusculatureCommand(self, command):
        try:
            self._send_musculature_command(command.muscle_commands)
        except:
            raise ArlArmException('Error while sending musculature command')

    def sendMuscleCommand(self, command):
        try:
            self._send_muscle_command(command)
        except:
            raise ArlArmException('Error while sending muscle command')

    def _send_musculature_command(self, commands):
        musculature_command = MusculatureCommand()
        musculature_command.header.stamp = rospy.get_rostime()
        musculature_command.header.frame_id = '0'
        for muscle_command in enumerate(commands):
            musculature_command.muscle_commands.append(self._generate_muscle_command_msg(muscle_command))
        self._musculature_command_publisher.publish(musculature_command)

    def _send_muscle_command(self, muscle_command):
        musculature_command = MusculatureCommand()
        musculature_command.header.stamp = rospy.get_rostime()
        musculature_command.header.frame_id = '0'
        musculature_command.muscle_commands.append(self._generate_muscle_command_msg(muscle_command))
        self._musculature_command_publisher.publish(musculature_command)

    def _generate_muscle_command_msg(self, muscle_command):
        muscle_command_msg = MuscleCommand()
        muscle_command_msg.name = muscle_command.name
        muscle_command_msg.pressure = muscle_command.pressure
        muscle_command_msg.activation = muscle_command.activation
        if muscle_command.control_mode == ArlMuscleControlMode.BY_ACTIVATION:
            muscle_command_msg.control_mode = MuscleCommand.CONTROL_MODE_BY_ACTIVATION
        else:
            muscle_command_msg.control_mode = MuscleCommand.CONTROL_MODE_BY_PRESSURE
        return muscle_command_msg


if __name__ == '__main__':
    print "Usage intented as library"
