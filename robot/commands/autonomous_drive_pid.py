from wpilib.command import Command
from wpilib import Timer
import wpilib.controller
from wpilib import SmartDashboard
import math

class AutonomousDrivePID(Command):
    """ This command drives the robot over a given distance with simple proportional control """
    # may need to use variables at some point ...

    def __init__(self, robot, setpoint=None, timeout=None, source=None):
        """The constructor"""
        #super().__init__()
        Command.__init__(self, name='autodrivepid')
        # Signal that we require ExampleSubsystem
        self.requires(robot.drivetrain)
        self.source = source
        self.k_dash = True

        # allow setpoint to be controlled by the dashboard
        if source == 'dashboard':
            setpoint = SmartDashboard.getNumber('distance', 1)
        else:
            if setpoint is None:
                setpoint = 2

        # wpilib PID controller apparently does not like negative setpoints, so use this to allow going backwards
        self.setpoint_sign = math.copysign(1, setpoint)
        self.setpoint = math.fabs(setpoint)  # correction for overshoot

        if timeout is None:
            self.timeout = 5
        else:
            self.timeout = timeout
        self.setTimeout(timeout)

        self.robot = robot
        self.has_finished = False
        self.error = 0

        # using these k-values
        self.kp = 1.8
        self.kd = 4.0
        self.ki = 0.00
        self.kperiod = 0.1
        self.tolerance = 0.1

        if self.k_dash:
            SmartDashboard.putNumber('kperiod_drive', self.kperiod)
            SmartDashboard.putNumber('ki_drive', self.ki)
            SmartDashboard.putNumber('kd_drive', self.kd)
            SmartDashboard.putNumber('kp_drive', self.kp)


    def corrected_setpoint(self, setpoint):
        """Found an excellent linear fit to the overshoot"""
        intercept, slope = 0.45, 1.56  # fit for kp=1.8, kd=0.2
        #return (setpoint - intercept) / slope
        return setpoint

    def initialize(self):
        """Called just before this Command runs the first time."""

        # allow setpoint to be controlled by the dashboard
        if self.source == 'dashboard':
            setpoint = SmartDashboard.getNumber('distance', 1)
            self.setpoint_sign = math.copysign(1, setpoint)
            self.setpoint = (math.fabs(setpoint))  # correction for overshoot

        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} with setpoint {self.setpoint} at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.getName()} with setpoint {self.setpoint} at {self.start_time} s **")
        self.has_finished = False

        # self.robot.drivetrain.reset_encoders()  # does not work in sim mode
        self.start_pos = self.robot.drivetrain.get_average_encoder_distance()

        if self.k_dash:
            self.kp, self.kd  = SmartDashboard.getNumber('kp', self.kp), SmartDashboard.getNumber('kd', self.kd)
            self.ki, self.period = SmartDashboard.getNumber('ki', self.ki), SmartDashboard.getNumber('kperiod', self.kperiod)

        self.controller = wpilib.controller.PIDController(self.kp, self.ki, self.kd, period=self.kperiod)
        self.controller.setSetpoint(self.corrected_setpoint(self.setpoint))
        self.controller.setTolerance(self.tolerance)
        self.controller.reset()

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.error = self.setpoint_sign*(self.robot.drivetrain.get_average_encoder_distance() - self.start_pos)
        output = self.controller.calculate(self.error)
        self.robot.drivetrain.arcade_drive(self.setpoint_sign * output, 0)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # somehow need to wait for the error level to get to a tolerance... request from drivetrain?
        if self.error > (self.setpoint - self.tolerance):
            self.has_finished = True
        return self.has_finished or self.isTimedOut()

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print(f"** {message} {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        self.robot.drivetrain.stop()

    def interrupted(self):
        self.end(message='Interrupted')

