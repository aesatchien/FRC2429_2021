#!/usr/bin/env python3
# Stripped-down version of 2020's Infinite Recharge robot for 2021's simulation and pathweaving

import wpilib
from wpilib import Timer
from commandbased import CommandBasedRobot
from wpilib.command import Scheduler
from commands.autonomous_ramsete import AutonomousRamsete
from commands.autonomous_home_slalom import AutonomousSlalom

# 2429-specific imports - need to import every subsystem you instantiate
from subsystems.drivetrain_sim import DriveTrainSim
from subsystems.drivetrain import DriveTrain
# ToDo: make a real drivetrain
from oi import OI

class Robot(CommandBasedRobot):
    """Main robot class"""

    def robotInit(self):
        """Robot-wide initialization code should go here"""
        super().__init__()

        if self.isReal():  # use the real drive train
            self.drivetrain = DriveTrain(self)
        else:  # use the simulated drive train
            self.drivetrain = DriveTrainSim(self)

        #self.drivetrain = DriveTrainSim(self)  # delete this later, the if statement messes up the IDE searches

        # oi MUST be created after all other subsystems since it uses them
        self.oi = OI(self)

        self.enabled_time = 0  # something is especially weird with the sim about this needing to be initialized in robotInit

        self.autonomousCommand = None  # initialize the placeholder command for autonomous

    def autonomousInit(self):
        """Called when autonomous mode is enabled"""
        self.enabled_time = Timer.getFPGATimestamp()
        self.autonomousCommand = AutonomousSlalom(self)
        self.autonomousCommand.start()

    def autonomousPeriodic(self):
        Scheduler.getInstance().run()

        '''        elapsed_time = Timer.getFPGATimestamp() - self.enabled_time
        if elapsed_time < 2.0:
            self.drivetrain.drive.arcadeDrive(1.0, -0.3)
        elif elapsed_time < 4.0:
            self.drivetrain.drive.arcadeDrive(1.0, 0.9)
        else:
            self.drivetrain.drive.arcadeDrive(0, 0)'''

    def teleopInit(self):
        """Called when teleop mode is enabled"""
        self.enabled_time = Timer.getFPGATimestamp()
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self):
        Scheduler.getInstance().run()

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        #wpilib.LiveWindow.run()
        pass

    def disabledInit(self):
        self.reset()

    def disabledPeriodic(self):
        """This function is called periodically while disabled."""
        self.log()

    def log(self):
        # worried about too much comm during the match
        pass

    def reset(self):
        pass

if __name__ == "__main__":
    wpilib.run(Robot)
