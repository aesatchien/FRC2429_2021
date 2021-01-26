# OI compatible with robotpy 2020
import wpilib
from wpilib import SmartDashboard, SendableChooser
from wpilib.command import JoystickButton, POVButton
from triggers.dpad import Dpad
from triggers.axis_button import AxisButton

# commands to bind
from commands.dpad_drive import DpadDrive
from commands.autonomous_rotate import AutonomousRotate
from commands.autonomous_drive_timed import AutonomousDriveTimed
from commands.autonomous_ramsete import AutonomousRamsete
from commands.autonomous_group import AutonomousGroup
from commands.autonomous_drive_pid import AutonomousDrivePID


class OI(object):
    """
    The operator interface of the robot.  Note we use competition_mode to determine if we will
    initialize a second joystick
    """
    def __init__(self, robot):
        self.robot = robot

        # Set single or double joystick mode
        self.competition_mode = False

        self.initialize_joystics()
        self.assign_buttons()

        self.initialize_dashboard()

    def assign_buttons(self):
        """Assign commands to buttons here"""
        # *** NOTE - THESE CAN FAIL IN COMPETITION IF YOU ARE RELYING ON A BUTTON TO BE HELD DOWN! ***
        self.dpad.whenPressed(DpadDrive(self.robot, button=self.dpad))

        # also bound to asdfg on the 2021 keyboard
        self.buttonA.whenPressed( AutonomousDriveTimed(self.robot, setpoint=2, timeout=3) )
        self.buttonB.whenPressed( AutonomousRotate(self.robot, setpoint=60, timeout=4, source='dashboard') )
        self.buttonX.whenPressed( AutonomousRotate(self.robot, setpoint=-60, timeout=4, source='dashboard', absolute=True) )
        self.buttonY.whenPressed( AutonomousDrivePID(self.robot, setpoint=2, timeout=3, source='dashboard') )
        self.buttonStart.whenPressed( AutonomousGroup(self.robot) )

    def initialize_joystics(self):
        """
        Assign all buttons on the driver and co-pilot's gamepads
        Does not need to be edited once written
        :return:
        """
        self.stick = wpilib.Joystick(0)
        self.buttonA = JoystickButton(self.stick, 1)
        self.buttonB = JoystickButton(self.stick, 2)
        self.buttonX = JoystickButton(self.stick, 3)
        self.buttonY = JoystickButton(self.stick, 4)
        self.buttonLB = JoystickButton(self.stick, 5)
        self.buttonRB = JoystickButton(self.stick, 6)
        self.buttonBack = JoystickButton(self.stick, 7)
        self.buttonStart = JoystickButton(self.stick, 8)
        self.dpad = Dpad(self.stick)

        # add/change bindings if we are using more than one joystick
        if self.competition_mode:
            self.co_stick = wpilib.Joystick(1)
            self.co_buttonA = JoystickButton(self.co_stick, 1)
            self.co_buttonB = JoystickButton(self.co_stick, 2)
            self.co_buttonX = JoystickButton(self.co_stick, 3)
            self.co_buttonY = JoystickButton(self.co_stick, 4)
            self.co_buttonLB = JoystickButton(self.co_stick, 5)
            self.co_buttonRB = JoystickButton(self.co_stick, 6)
            self.co_buttonBack = JoystickButton(self.co_stick, 7)
            self.co_buttonStart = JoystickButton(self.co_stick, 8)
            self.co_povButtonUp = POVButton(self.co_stick, 0)
            self.co_povButtonDown = POVButton(self.co_stick, 180)
            self.co_povButtonRight = POVButton(self.co_stick, 90)
            self.co_povButtonLeft = POVButton(self.co_stick, 270)
            self.co_axisButtonLT = AxisButton(self.co_stick, 2)
            self.co_axisButtonRT = AxisButton(self.co_stick, 3)

    def initialize_dashboard(self):
        # dummy setpoints to speed up testing from the dashboard

        SmartDashboard.putNumber('distance', 2.0)
        SmartDashboard.putNumber('angle', 60)

        self.drive_fwd_command =  AutonomousDriveTimed(self.robot, setpoint=2, timeout=3)
        self.rotate_command = AutonomousRotate(self.robot, setpoint=45, timeout=3, source='dashboard')
        self.autonomous_test_command = AutonomousGroup(self.robot)
        self.autonomous_test_ramsete_command = AutonomousRamsete(self.robot)

        # set up the dashboard chooser for the autonomous options
        self.routine_chooser = SendableChooser()
        routes = ['Slalom', 'Barrel Roll', 'Bounce']
        for ix, position in enumerate(routes):
            if ix == 0:
                self.routine_chooser.setDefaultOption(position, position)
            else:
                self.routine_chooser.addOption(position, position)
        wpilib.SmartDashboard.putData('Autonomous Routine', self.routine_chooser)