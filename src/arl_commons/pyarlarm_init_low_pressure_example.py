#!/usr/bin/env python
from pyarlarm import ArlArm, ArlMusculature, ArlMuscleState, ArlMuscleCommand, ArlMuscleControlMode, ArlArmException
from time import sleep


class Example:
    def __init__(self):
        self._muscle_names = [('muscle_' + str(idx)) for idx in range(1, 29)]

    def run(self):
        arm = ArlArm()
        musculature = {}

        for muscle_idx in range(28):
            muscle_comm = ArlMuscleCommand()
            muscle_comm.name = self._muscle_names[muscle_idx]
            muscle_comm.pressure = 4000.0
            muscle_comm.control_mode = ArlMuscleControlMode.BY_PRESSURE
            # muscle_comm.activation = -0.3
            # muscle_comm.control_mode = ArlMuscleControlMode.BY_ACTIVATION
            musculature[self._muscle_names[muscle_idx]] = muscle_comm

        commands = ArlMusculature()
        commands.muscle_commands = musculature
        arm.sendMusculatureCommand(commands)

        arm.shutdown()

if __name__ == '__main__':
    example = Example()
    example.run()
