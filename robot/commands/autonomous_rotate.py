from wpilib.command import Command
from wpilib import Timer
import math
from wpilib import SmartDashboard
from networktables import NetworkTables

class AutonomousRotate(Command):
    """
    This command rotates the robot over a given angle with simple proportional
    control
    """
    def __init__(self, robot, setpoint=None, timeout=None, source=None):
        """The constructor"""
        Command.__init__(self, name='autonomousrotate')
        self.requires(robot.drivetrain)
        self.setpoint = setpoint
        self.source = source
        if timeout is None:
            self.setTimeout(3)
        else:
            self.setTimeout(timeout)
        self.robot = robot
        self.tolerance = 1
        self.kp = 0.3
        self.kd = 0.1
        self.kf = 0.1
        self.start_angle = 0
        self.error = 0
        self.power = 0
        self.max_power = 0.5
        self.prev_error = 0

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)

        if self.source == 'dashboard':
            self.setpoint = SmartDashboard.getNumber('angle', 1)
        else:
            pass

        print("\n" + f"** Started {self.getName()} with setpoint {self.setpoint} at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.getName()} with setpoint {self.setpoint} at {self.start_time} s **")
        self.start_angle = self.robot.drivetrain.navx.getAngle()
        self.error = 0
        self.prev_error = 0

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.error = self.setpoint - (self.robot.drivetrain.navx.getAngle()-self.start_angle)
        self.power = self.kp * self.error + self.kf * math.copysign(1, self.error) + self.kd * (self.error - self.prev_error) / 0.02
        self.prev_error = self.error
        if self.power >0:
            self.power = min(self.max_power, self.power)
        else:
            self.power = max(-self.max_power, self.power)
        #self.robot.drivetrain.smooth_drive(0,-self.power)
        self.robot.drivetrain.arcade_drive(thrust=0, twist=self.power)
        SmartDashboard.putNumber("error", self.error)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # I know I could do this with a math.copysign, but this is more readable
        if self.setpoint > 0:
            return (self.setpoint - (self.robot.drivetrain.navx.getAngle()-self.start_angle)) <= self.tolerance or self.isTimedOut()
        else:
            return (self.setpoint - (self.robot.drivetrain.navx.getAngle() - self.start_angle)) >= -self.tolerance or self.isTimedOut()

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** {message} {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.getName()} at {end_time} s after {round(end_time - self.start_time, 1)} s **")
        self.robot.drivetrain.stop()

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end(message='Interrupted')
