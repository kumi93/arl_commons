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
        command.pressure = 999
        command.control_mode = ArlMuscleControlMode.BY_PRESSURE
        arm.sendMuscleCommand(command)

        count = 0
        while arm.ok:
            try:
                command.pressure = 100 * count
                arm.sendMuscleCommand(command)
                print str(arm.receiveMuscleState('muscle_25').desired_pressure)
                count +=1
                sleep(1)
            except ArlArmException as e:
                pass
        arm.shutdown()


if __name__ == '__main__':
    example = Example()
    example.run()
