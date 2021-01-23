from wpilib.command import Command

class DriveByJoystick(Command):
    """
    This allows Logitech gamepad to drive the robot. It is always running
    except when interrupted by another command.
    """

    def __init__(self, robot):
        Command.__init__(self, name='DriveByJoystick')
        self.requires(robot.drivetrain)
        self.robot = robot

    def initialize(self):
        pass


    def execute(self):
        self.robot.drivetrain.drive.arcadeDrive(-self.robot.oi.stick.getRawAxis(1), 0.75*self.robot.oi.stick.getRawAxis(0))


    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return False


    def end(self):
        self.robot.drivetrain.drive.arcadeDrive(0,0)


    def interrupted(self):
        self.robot.drivetrain.drive.arcadeDrive(0,0)