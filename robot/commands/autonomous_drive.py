from wpilib.command import Command
from wpilib import Timer
from wpilib import SmartDashboard
from networktables import NetworkTables

class AutonomousDrive(Command):
    """
    This command drives the robot over a given distance with simple proportional control

    """
    # may need to use variables at some point ...
    tolerance = 0.5

    def __init__(self, robot, setpoint=None, timeout=None, source=None):
        """The constructor"""
        #super().__init__()
        Command.__init__(self, name='autonomousdrive')
        # Signal that we require ExampleSubsystem
        self.requires(robot.drivetrain)
        if setpoint is None:
            setpoint = 2
        else:
            self.setpoint = setpoint
        if timeout is None:
            self.setTimeout(5)
            self.timeout = 5
        else:
            self.setTimeout(timeout)
            self.timeout = timeout
        self.robot = robot
        self.tolerance = 0.1
        self.has_finished = False

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} with setpoint {self.setpoint} at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.getName()} with setpoint {self.setpoint} at {self.start_time} s **")
        self.has_finished = False
        # self.robot.drivetrain.reset_encoders()  # does not work in sim mode
        self.start_pos = self.robot.drivetrain.get_average_encoder_distance()

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.robot.drivetrain.arcade_drive(1,0)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # somehow need to wait for the error level to get to a tolerance... request from drivetrain?
        if (self.robot.drivetrain.get_average_encoder_distance() - self.start_pos) > (self.setpoint - self.tolerance):
            self.has_finished = True
        return self.has_finished or self.isTimedOut()

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** {message} {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        self.robot.drivetrain.stop()

    def interrupted(self):
        self.end(message='Interrupted')

