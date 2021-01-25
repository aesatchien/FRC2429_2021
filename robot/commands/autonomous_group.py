# just leaving this from pacgoat as an example for now
from wpilib.command import CommandGroup
from commands.autonomous_drive import AutonomousDrive
from commands.autonomous_rotate import AutonomousRotate


class AutonomousGroup(CommandGroup):
    """
    allows for stringing together autonomous commands
    """

    def __init__(self, robot, timeout=None):
        CommandGroup.__init__(self, name='AutonomousGroup')
        # slalom
        self.addSequential(AutonomousDrive(robot, setpoint=1, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=-85, timeout=5))
        self.addSequential(AutonomousDrive(robot, setpoint=1.5, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=85, timeout=5))
        self.addSequential(AutonomousDrive(robot, setpoint=2, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=85, timeout=5))
        self.addSequential(AutonomousDrive(robot, setpoint=1.5, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=-85, timeout=5))
        self.addSequential(AutonomousDrive(robot, setpoint=1.5, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=-85, timeout=5))
        self.addSequential(AutonomousDrive(robot, setpoint=1.5, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=-85, timeout=5))


