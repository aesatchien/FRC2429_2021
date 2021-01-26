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

import subsystems.drive_constants as drive_constants

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
        self.counter = 0
        self.x, self.y = 0, 0

        # --------  INITIALIZE HARDWARE  ---------------
        self.l_motor = simlib.PWMSim(1)
        self.r_motor = simlib.PWMSim(3)

        # NavX (SPI interface) - no idea why the "4" is there, just using it as given
        self.navx = simlib.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")

        # set up two simulated encoders to see how they effect the robot
        self.l_distance, self.r_distance = 0, 0
        self.l_distance_old, self.r_distance_old = 0, 0
        if version == '2020':
            self.r_encoder = simlib.EncoderSim(0)
            self.l_encoder = simlib.EncoderSim(1)
        else:
            # in 2021 you have to pass the actual object
            self.r_encoder = simlib.EncoderSim.createForChannel(0)
            self.l_encoder = simlib.EncoderSim.createForChannel(2)
        # update units from drive constants
        self.l_encoder.setDistancePerPulse(drive_constants.encoder_distance_per_pulse_m)
        self.r_encoder.setDistancePerPulse(drive_constants.encoder_distance_per_pulse_m)


        # NavX (SPI interface) - no idea why the "4" is there, just using it as given
        self.navx = simlib.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")

        # --------  INITIALIZE FIELD SETTINGS  ---------------

        # keep us on the field - set x,y limits for driving
        field_size = 'home'
        if field_size == 'competition':
            self.x_limit, self.y_limit = 15.97, 8.21  # meters for a 52.4x26.9' field
        else: # at home autonomous challenge for 2021
            self.x_limit, self.y_limit = 9.144, 9.144 / 2  # meters for a 30x15' field

        # Set our position on the field
        self.x = 1.16 #2  # x y rot position data calculated from the pose
        self.y = 0.74 # 8.21/2
        initial_pose = geo.Pose2d(0, 0, geo.Rotation2d(0))
        final_pose = geo.Pose2d(self.x, self.y, geo.Rotation2d(0))
        initial_position_transform = geo.Transform2d(initial_pose, final_pose)
        self.physics_controller.move_robot(initial_position_transform)

        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        # fmt: off
        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
            110 * units.lbs,                    # robot mass
            drive_constants.gear_ratio,         # drivetrain gear ratio
            2,                                  # motors per side
            22 * units.inch,                    # robot wheelbase
            23 * units.inch + bumper_width * 2, # robot width
            32 * units.inch + bumper_width * 2, # robot length
            8 * units.inch,                     # wheel diameter
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

        # Simulate the drivetrain and update encoders
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed()

        transform = self.drivetrain.calculate(l_motor, r_motor, tm_diff)
        pose = self.physics_controller.move_robot(transform) # includes inertia

        # keep us on the simulated field - reverse the transform if we try to go out of bounds
        if (pose.translation().x < 0 or pose.translation().x > self.x_limit or
                pose.translation().y < 0 or pose.translation().y > self.y_limit):
            curr_x, curr_y = transform.translation().x, transform.translation().y
            # in 2021 library they added an inverse() to transforms so this could all be one line
            new_transform = geo.Transform2d(geo.Translation2d(-curr_x, curr_y), transform.rotation())
            pose = self.physics_controller.move_robot(new_transform)

        # Update encoders - need use the pose so it updates properly for coasting to a stop
        self.l_distance += l_motor * 3.0*tm_diff
        self.r_distance += -r_motor * 3.0*tm_diff

        if version == '2021':
            self.l_encoder.setDistance(self.l_distance)
            self.r_encoder.setDistance(self.r_distance)
            self.l_encoder.setRate((self.l_distance-self.l_distance_old)/tm_diff)
            self.r_encoder.setRate((self.r_distance-self.r_distance_old)/tm_diff)
            self.l_distance_old = self.l_distance
            self.r_distance_old = self.r_distance
        else:
            self.l_encoder.setCount(int(self.l_distance / drive_constants.encoder_distance_per_pulse_m))
            self.r_encoder.setCount(int(self.r_distance / drive_constants.encoder_distance_per_pulse_m))


        self.x, self.y = pose.translation().x, pose.translation().y
        # Update the navx gyro simulation
        # -> FRC gyros like NavX are positive clockwise, but the returned pose is positive counter-clockwise
        self.navx_yaw.set(-pose.rotation().degrees())

