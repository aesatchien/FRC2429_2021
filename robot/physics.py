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
from wpilib import SmartDashboard

import subsystems.drive_constants as drive_constants
import wpimath.geometry as geo
import wpilib.simulation as simlib  # 2021 name for the simulation library


class PhysicsEngine:
    """
        Simulates a motor moving something that strikes two limit switches,
        one on each end of the track. Obviously, this is not particularly
        realistic, but it's good enough to illustrate the point
    """

    sparkmax = False

    def __init__(self, physics_controller: PhysicsInterface):

        self.physics_controller = physics_controller  # mandatory

        self.counter = 0
        self.x, self.y = 0, 0
        self.obstacles = 'slalom'  # initialize only, this is read from the dash periodically
        self.hood_position = 30

        # --------  INITIALIZE HARDWARE  ---------------

        if self.sparkmax:  # use the sparkmax drivetrain objects, access their position and velocity attributes
            self.l_spark = simlib.SimDeviceSim('SPARK MAX [3]')
            self.r_spark = simlib.SimDeviceSim('SPARK MAX [1]')
            print(f'** SparkMax allows access to: **\n{self.r_spark.enumerateValues()} ')
            self.l_spark_position = self.l_spark.getDouble('Position')
            self.r_spark_position = self.r_spark.getDouble('Position')
            self.l_spark_velocity = self.l_spark.getDouble('Velocity')
            self.r_spark_velocity = self.r_spark.getDouble('Velocity')
            self.l_spark_output = self.l_spark.getDouble('Applied Output')
            self.r_spark_output = self.r_spark.getDouble('Applied Output')

        else:  # use the generic wpilib motors and encoders
            self.l_motor = simlib.PWMSim(1)
            self.r_motor = simlib.PWMSim(3)
            self.l_encoder = simlib.EncoderSim.createForChannel(0)
            self.r_encoder = simlib.EncoderSim.createForChannel(2)

            # update units from drive constants
            self.l_encoder.setDistancePerPulse(drive_constants.k_encoder_distance_per_pulse_m)
            self.r_encoder.setDistancePerPulse(drive_constants.k_encoder_distance_per_pulse_m)

        # hood IO
        self.hood_encoder = simlib.EncoderSim.createForChannel(4)
        self.hood_encoder.setDistancePerPulse(1/1024)
        self.limit_low = simlib.DIOSim(6)
        self.limit_hi = simlib.DIOSim(7)
        self.hood_motor = simlib.PWMSim(5)

        # NavX (SPI interface) - no idea why the "4" is there, seems to be the default name generated by the navx code
        self.navx = simlib.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")

        # set up two simulated encoders to see how they effect the robot
        self.l_distance, self.r_distance = 0, 0
        self.l_distance_old, self.r_distance_old = 0, 0  # used for calculating encoder rates


        # --------  INITIALIZE FIELD SETTINGS  ---------------

        # keep us on the field - set x,y limits for driving
        field_size = 'home'
        if field_size == 'competition':
            self.x_limit, self.y_limit = 15.97+0.5, 8.21+0.5  # meters for a 52.4x26.9' field PLUS 0.5 meter border
            self.x, self.y = 2, 8.21/2
        else: # at home autonomous challenge for 2021
            self.x_limit, self.y_limit = 9.114, 4.572  # meters for a 30x15' DO NOT INCLUDE BORDER.  To that with the imgui.ini.
                    # Set our position on the field
            position = 'slalom'  # set this to put the robot on the field
            if position == 'slalom':
                self.x, self.y = 1.1, 0.68
            elif position  == 'barrel':
                self.x, self.y = 1.1, 2.3-.25
            elif position == 'bounce':
                self.x, self.y = 1.1, 2.5-.25
            else:
                self.x, self.y = 0, 0

        self.field_offset = geo.Transform2d(geo.Translation2d(0.25,0.25), geo.Rotation2d(0))

        # is all this necessary?
        initial_pose = geo.Pose2d(0, 0, geo.Rotation2d(0))
        final_pose = geo.Pose2d(self.x, self.y, geo.Rotation2d(0))
        initial_position_transform = geo.Transform2d(initial_pose, final_pose)
        self.physics_controller.move_robot(initial_position_transform)

        # --------  INITIALIZE ROBOT MODEL  ---------------
        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        # fmt: off
        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
            110 * units.lbs,                    # robot mass
            drive_constants.k_gear_ratio,         # drivetrain gear ratio
            2,                                  # motors per side
            27 * units.inch,                    # robot wheelbase, 27 in = 0.69 meters
            27 * units.inch + bumper_width * 2, # robot width
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
        if self.sparkmax:
            #self.l_spark_output.set(0.333) # need the -1 to 1 motor value, and I have to fake it
            #self.r_spark_output.set(0.999)
            l_motor = self.l_spark_output.get()
            r_motor = self.r_spark_output.get()
        else:
            l_motor = self.l_motor.getSpeed()  # these are PWM speeds of -1 to 1
            r_motor = self.r_motor.getSpeed()   #  going forward, left should be positive and right is negative

        # get a new location based on motor movement
        transform = self.drivetrain.calculate(l_motor, r_motor, tm_diff)
        pose = self.physics_controller.move_robot(transform)  # includes inertia

        # keep us on the simulated field - reverse the transform if we try to go out of bounds
        sim_padding = 0.25 # 0.25  # let us go a bit outside but not get lost
        bad_move = False  # see if we are out of bounds or hitting a barrier
        if (pose.translation().x < -sim_padding or pose.translation().x > self.x_limit + sim_padding or
                pose.translation().y < -sim_padding or pose.translation().y > self.y_limit + sim_padding):
            bad_move = True

        # allowing the user to change the obstacles
        pylon_points = []
        if 'slalom' in self.obstacles:
            pylon_points = drive_constants.slalom_points
        if 'barrel' in self.obstacles:
            pylon_points = drive_constants.barrel_points
        if 'bounce' in self.obstacles:
            pylon_points = drive_constants.bounce_points
        if any([drive_constants.distance(pose, i)
                < drive_constants.k_track_width_meters / 2 for i in pylon_points]):
            bad_move = True

        # this resets the move if we are hitting a barrier
        # ToDo: stop the motion as well.  Perhaps in addition to moving back we should redo transform w/ no motor speed
        if bad_move:
            curr_x, curr_y = transform.translation().x, transform.translation().y
            # in 2021 library they added an inverse() to transforms so this could all be one line
            new_transform = geo.Transform2d(geo.Translation2d(-curr_x, curr_y), transform.rotation())
            pose = self.physics_controller.move_robot(new_transform)

        # Update encoders - need use the pose so it updates properly for coasting to a stop
        # ToDo: get the actual values, probably from wheelspeeds?
        # have to only work with deltas otherwise we can't reset the encoder from the real code
        # damn it, the SparkMax and the standard encoders have different calls

        if self.sparkmax:
            self.l_spark_position.set(self.l_spark_position.get() + (self.drivetrain.l_position - self.l_distance_old) * 0.3105)
            self.r_spark_position.set(self.r_spark_position.get() + (self.drivetrain.r_position - self.r_distance_old) * 0.3105)
            self.l_spark_velocity.set(0.31 * (self.drivetrain.l_position - self.l_distance_old) / tm_diff)
            self.r_spark_velocity.set(0.31 * (self.drivetrain.r_position - self.r_distance_old) / tm_diff)
        else:
            self.l_encoder.setDistance(self.l_encoder.getDistance() + (self.drivetrain.l_position-self.l_distance_old) * 0.3105)
            self.r_encoder.setDistance(self.r_encoder.getDistance() - (self.drivetrain.r_position-self.r_distance_old) * 0.3105)  # negative going forward
            self.l_encoder.setRate(0.31*(self.drivetrain.l_position -self.l_distance_old)/tm_diff)
            self.r_encoder.setRate(-0.31*(self.drivetrain.r_position -self.r_distance_old)/tm_diff)  # needs to be negitive going fwd

        self.l_distance_old = self.drivetrain.l_position
        self.r_distance_old = self.drivetrain.r_position

        self.x, self.y = pose.translation().x, pose.translation().y
        # Update the navx gyro simulation
        # -> FRC gyros like NavX are positive clockwise, but the returned pose is positive counter-clockwise
        self.navx_yaw.set(-pose.rotation().degrees())


# ---------------- HOOD ELEVATOR UPDATES ---------------------------
        # update 'position' (use tm_diff so the rate is constant) - this is for simulating an elevator, arm etc w/ limit switches
        self.hood_position += self.hood_motor.getSpeed() * tm_diff * 30 - 0.05 # inserting gravity!

        # update limit switches based on position
        if self.hood_position <= 30:
            switch1 = True
            switch2 = False
            if self.hood_position < 29.5:  # don't let gravity sag too much
                self.hood_position = 29.5
        elif self.hood_position > 50:
            switch1 = False
            switch2 = True

        else:
            switch1 = False
            switch2 = False

        # set values here - nice way to emulate the robot's state
        self.limit_low.setValue(switch1)
        self.limit_hi.setValue(switch2)
        self.hood_encoder.setDistance(round(self.hood_position, 2))


        self.counter += 1
        if self.counter % 5 == 0:
            SmartDashboard.putNumber('field_x', round(self.x, 2))
            SmartDashboard.putNumber('field_y', round(self.y, 2))
            SmartDashboard.putNumber('hood', round(self.hood_position, 1))

        if self.counter % 50 == 0:
            self.obstacles = SmartDashboard.getData('obstacles').getSelected()  # read the obstacles from the dash