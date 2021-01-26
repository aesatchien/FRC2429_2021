# just leaving this from pacgoat as an example for now
from wpilib.command import CommandGroup
from commands.autonomous_drive import AutonomousDrive
from commands.autonomous_rotate import AutonomousRotate
from commands.autonomous_drive_pid import AutonomousDrivePID


class AutonomousGroup(CommandGroup):
    """
    allows for stringing together autonomous commands
    """

    def __init__(self, robot, timeout=None):
        CommandGroup.__init__(self, name='AutonomousGroup')
        # slalom
        scale = 0.9
        self.addSequential(AutonomousDrivePID(robot, setpoint=1*scale, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=-55, timeout=5))
        self.addSequential(AutonomousDrivePID(robot, setpoint=1.8*scale, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=55, timeout=5))
        self.addSequential(AutonomousDrivePID(robot, setpoint=2*scale, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=55, timeout=5))
        self.addSequential(AutonomousDrivePID(robot, setpoint=1.8*scale, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=-45, timeout=5))
        self.addSequential(AutonomousDrivePID(robot, setpoint=1*scale, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=-85, timeout=5))
        self.addSequential(AutonomousDrivePID(robot, setpoint=1.5*scale, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=-95, timeout=5))
        self.addSequential(AutonomousDrivePID(robot, setpoint=1.5*scale, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=-85, timeout=5))
        self.addSequential(AutonomousDrivePID(robot, setpoint=1.4 * scale, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=80, timeout=5))
        self.addSequential(AutonomousDrivePID(robot, setpoint=2.6 * scale, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=50, timeout=5))
        self.addSequential(AutonomousDrivePID(robot, setpoint=1.6 * scale, timeout=6))
        self.addSequential(AutonomousRotate(robot, setpoint=-60, timeout=5))
        self.addSequential(AutonomousDrivePID(robot, setpoint=1.0 * scale, timeout=6))

