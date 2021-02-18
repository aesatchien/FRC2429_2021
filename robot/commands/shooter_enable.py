from wpilib.command import Command
from wpilib import Timer


class ShooterEnable(Command):

    def __init__(self, robot, button=None, timeout=None, velocity = 0):
        Command.__init__(self, name='Shooter_enable')
        self.robot = robot
        self.timeout = timeout
        self.button = button
        self.velocity = 0.5

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} with velocity {self.velocity} at {self.start_time} s **", flush=True)
    

    def execute(self):
        self.robot.shooter.set_feed_motor(self.velocity)

    def end(self):
        """Called once after isFinished returns true"""
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end()

