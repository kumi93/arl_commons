#!/usr/bin/env python
from pyarlarm import ArlArm, ArlMusculature, ArlMuscleState, ArlMuscleCommand, ArlMuscleControlMode, ArlArmException
from time import sleep


class Example:
    def __init__(self):
        pass

    def run(self):
        arm = ArlArm()
        command = ArlMuscleCommand()
        command.name = 'muscle_25'
        command.pressure = 3500.0 #(up to 10000.0)
        command.control_mode = ArlMuscleControlMode.BY_PRESSURE
        arm.sendMuscleCommand(command)

        while arm.ok:
            try:
                command.pressure += 1000
                arm.sendMuscleCommand(command)
                print str(arm.receiveMuscleState('muscle_25').desired_pressure)
                sleep(1)
            except ArlArmException as e:
                pass
        arm.shutdown()


if __name__ == '__main__':
    example = Example()
    example.run()
