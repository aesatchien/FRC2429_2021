from wpilib.command import Command
from wpilib import Timer
import wpilib.controller

class ShooterEnable(Command):

    def __init__(self, robot, button=None, timeout=None, velocity = 0):
        Command.__init__(self, name='Shooter_enable')
        self.robot = robot
        self.timeout = timeout
        self.button = button
        
        #pid variables
        self.P = 1
        self.I = 1
        self.D = 1
        self.period = 0.01
        self.tolererance = 0.1
        self.setpoint = 0.5 #desired velocity
        self.error = 0

        #pid controllers
        self.flywheel_PID_controller = wpilib.controller.PIDController(self.P, self.I, self.D, self.period)
        self.flywheel_PID_controller.setSetpoint(self.setpoint) #desired velocity
        self.flywheel_PID_controller.setTolerance(self.tolererance)
        self.flywheel_PID_controller.reset()
        

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} with velocity {self.velocity} at {self.start_time} s **", flush=True)
    

    def execute(self):
        output = self.flywheel_PID_controller.calculate(self.robot.shooter.get_velocity(), self.setpoint)
        self.robot.shooter.set_flywheel(output)
        if(self.flywheel_PID_controller.atSetpoint()):
            self.robot.shooter.is_enabled(True)


    def end(self):
        """Called once after isFinished returns true"""
        print("\n" + f"** Ended {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")


    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end()

