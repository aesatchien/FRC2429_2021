from wpilib.command import Command
from wpilib import Timer
import wpilib.controller

class ShooterAdjust(Command):
 
    
    def __init__(self, robot, button=None, timeout=None):
        Command.__init__(self, name='Shooter_adjust')
        self.robot = robot
        self.timeout = timeout
        self.button = button
        self.max_power = 0.75


    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True) 

    def execute(self):
       pass
    
    def end(self):
        """Called once after isFinished returns true"""
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end()

