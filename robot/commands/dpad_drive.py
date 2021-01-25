from wpilib.command import Command
from wpilib import Timer, SmartDashboard
import math

class DpadDrive(Command):
    """
    This allows Logitech gamepad's dpad to drive the robot. It overrides the stick.
    Change the drive_power and twist_power variables to change how it reacts
    """

    def __init__(self, robot, button):
        Command.__init__(self, name='DpadDrive')
        self.requires(robot.drivetrain)
        self.robot = robot
        self.button = button

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp(), 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.robot.enabled_time} s **")
        #self.heading = self.robot.navigation.get_angle()

    def execute(self):
        """
        Called repeatedly when this Command is scheduled to run
        Should not have to change this if it works - just change variables above to tweak speeds and correction
        """
        # easy to correct for heading drift - we know we're trying to drive straight if we keep previous angle heading ...
        angle = self.button.angle() * math.pi / 180.
        thrust, twist = math.cos(angle), 0.75*math.sin(angle)


        if self.robot.isReal():
            if self.mode == 'velocity':
                self.robot.drivetrain.mecanum_velocity_cartesian(thrust=thrust, strafe=0, z_rotation=twist)
            else:
                self.robot.drivetrain.smooth_drive(thrust=thrust, strafe=0, twist=twist)
        else:
            # simulation
            self.robot.drivetrain.arcade_drive(thrust, twist)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return not self.button.get()

    def end(self):
        """Called once after isFinished returns true"""
        self.robot.drivetrain.stop()
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.robot.drivetrain.stop()
        print("\n" + f"** Interrupted {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
