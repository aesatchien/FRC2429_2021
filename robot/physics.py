#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units
import wpilib.geometry as geo

import wpilib # need some other way of checking version w/o importing unneeded stuff
version = wpilib.__version__[0:4]
print(f'*** Version is {version} ***')
if version == '2020':
    import hal.simulation as simlib # 2020
elif version == '2021':
    import wpilib.simulation as simlib #2021


class PhysicsEngine:
    """
        Simulates a motor moving something that strikes two limit switches,
        one on each end of the track. Obviously, this is not particularly
        realistic, but it's good enough to illustrate the point
    """

    def __init__(self, physics_controller: PhysicsInterface):

        self.physics_controller = physics_controller

        # Making the transition from 2020 to 2021 libraries
        self.dio1 = simlib.DIOSim(1)
        self.dio2 = simlib.DIOSim(2)
        self.motor = simlib.PWMSim(4)
        self.l_motor = simlib.PWMSim(1)
        self.r_motor = simlib.PWMSim(2)

        # Gyro
        self.gyro = simlib.AnalogGyroSim(1)
        if version == '2021':
            self.ain2 = simlib.AnalogInputSim(2)
        elif version == '2020':
            self.ain2 = simlib.AnalogInSim(2)

        self.position = 0

        # cjh keep us on the field - still need to figure out the corner case and mecanum case (may need to flip y?)
        field_size = 'competition'
        if field_size == 'competition':
            self.x_limit, self.y_limit = 15.97, 8.21  # meters for a 52.4x26.9' field
        else: # at home autonomous challenge for 2021
            self.x_limit, self.y_limit = 9.144, 9.144 / 2  # meters for a 30x15' field




        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        # fmt: off
        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
            110 * units.lbs,                    # robot mass
            10.71,                              # drivetrain gear ratio
            2,                                  # motors per side
            22 * units.inch,                    # robot wheelbase
            23 * units.inch + bumper_width * 2, # robot width
            32 * units.inch + bumper_width * 2, # robot length
            6 * units.inch,                     # wheel diameter
        )
        # fmt: on

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
            Called when the simulation parameters for the program need to be
            updated.
            
            :param now: The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
        """

        # Simulate the drivetrain
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed()

        transform = self.drivetrain.calculate(l_motor, r_motor, tm_diff)
        pose = self.physics_controller.move_robot(transform)

        if pose.translation().x < 0 or pose.translation().x > self.x_limit:
            curr_x = transform.translation().x
            curr_y = transform.translation().y
            new_transform = geo.Transform2d(geo.Translation2d(-curr_x, curr_y), transform.rotation())
            self.physics_controller.move_robot(new_transform)

        if pose.translation().y < 0 or pose.translation().y > self.y_limit:
            curr_x = transform.translation().x
            curr_y = transform.translation().y
            new_transform = geo.Transform2d(geo.Translation2d(-curr_x, curr_y), transform.rotation())
            self.physics_controller.move_robot(new_transform)




# -----------  DELETE THIS STUFF AFTER EXPLAINING -------------------
        # Update the gyro simulation
        # -> FRC gyros are positive clockwise, but the returned pose is positive
        #    counter-clockwise
        self.gyro.setAngle(-pose.rotation().degrees())

        # update position (use tm_diff so the rate is constant)
        self.position += self.motor.getSpeed() * tm_diff * 3

        # update limit switches based on position
        if self.position <= 0:
            switch1 = True
            switch2 = False

        elif self.position > 5:
            switch1 = False
            switch2 = True

        else:
            switch1 = False
            switch2 = False

        # set values here
        self.dio1.setValue(switch1)
        self.dio2.setValue(switch2)
        self.ain2.setVoltage(self.position)
