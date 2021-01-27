# a group of commands strung together autonomously
from wpilib.command import CommandGroup
from commands.autonomous_rotate import AutonomousRotate
from commands.autonomous_drive_pid import AutonomousDrivePID


class AutonomousBounce(CommandGroup):
    """ allows for stringing together autonomous commands """

    def __init__(self, robot, timeout=None):
        CommandGroup.__init__(self, name='AutonomousBounce')
        # slalom
        relative_angles = False
        if relative_angles:
            self.addSequential(AutonomousDrivePID(robot, setpoint=0.8, timeout=3))  # initial move
            self.addSequential(AutonomousRotate(robot, setpoint=-55, timeout=3))  # first turn

        else:  # use absolute heading
            # slalom with absolute angles E=0, S=90, W=180, N=-90 or 270
            diagonal_distance = 1.8
            self.addSequential(AutonomousDrivePID(robot, setpoint=1, timeout=3))  # initial move
            self.addSequential(AutonomousRotate(robot, setpoint=-90, absolute=True, timeout=3))  # first turn
            self.addSequential(AutonomousDrivePID(robot, setpoint=4, timeout=3))  # drive to top


