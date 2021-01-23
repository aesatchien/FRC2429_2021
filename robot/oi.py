# OI compatible with robotpy 2020
import wpilib
from wpilib import SmartDashboard, SendableChooser
from wpilib.command import JoystickButton

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

    def assign_buttons(self):
        """Assign commands to buttons here"""
        pass

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