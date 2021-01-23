
from wpilib.command import Subsystem
from wpilib import Jaguar
import wpilib.drive
from commands.drive_by_joystick import DriveByJoystick

class DriveTrainDemo(Subsystem):
    pass

    def __init__(self, robot):
        super().__init__("drivetraindemo")
        self.robot = robot

        self.l_motor = Jaguar(1)
        self.r_motor = Jaguar(2)
        self.drive = wpilib.drive.DifferentialDrive(self.l_motor, self.r_motor)

    def initDefaultCommand(self):
        """
        When other commands aren't using the drivetrain, allow arcade drive with the joystick.
        """
        self.setDefaultCommand(DriveByJoystick(self.robot))