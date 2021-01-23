#!/usr/bin/env python3

import wpilib
from commandbased import CommandBasedRobot
from wpilib.command import Scheduler

# 2429-specific imports - need to import every subsystem you instantiate
from subsystems.drivetrain_demo import DriveTrainDemo
from oi import OI

class Robot(CommandBasedRobot):
    """Main robot class"""

    def robotInit(self):
        """Robot-wide initialization code should go here"""
        super().__init__()

        self.drivetrain = DriveTrainDemo(self)
        self.oi = OI(self)

        if self.isReal():
            print('*** Robot is real ***')
        else:
            print('*** Robot is not real.  Running simulation ***')

        # Position gets automatically updated as robot moves
        self.gyro = wpilib.AnalogGyro(1)
        self.limit1 = wpilib.DigitalInput(1)
        self.limit2 = wpilib.DigitalInput(2)
        self.motor = wpilib.Jaguar(4)

        self.position = wpilib.AnalogInput(2)

        self.timer = wpilib.Timer()

    def autonomousInit(self):
        """Called when autonomous mode is enabled"""

        self.timer.reset()
        self.timer.start()

    def autonomousPeriodic(self):
        Scheduler.getInstance().run()
        if self.timer.get() < 2.0:
            self.drivetrain.drive.arcadeDrive(1.0, -0.3)
        elif self.timer.get() < 4.0:
            self.drivetrain.drive.arcadeDrive(1.0, 0.9)
        else:
            self.drivetrain.drive.arcadeDrive(0, 0)

    def TeleopInit(self):
        """Called when teleop mode is enabled"""
        self.timer.reset()
        self.timer.start()
        print(f'Enabling teleop at time = {round(self.timer.get(),1)}')

    def teleopPeriodic(self):
        Scheduler.getInstance().run()

        # Move a motor with a Joystick
        y = -self.oi.stick.getRawAxis(5)

        # stop movement backwards when 1 is on
        if self.limit1.get():
            y = max(0, y)

        # stop movement forwards when 2 is on
        if self.limit2.get():
            y = min(0, y)

        self.motor.set(y)


if __name__ == "__main__":
    wpilib.run(Robot)
