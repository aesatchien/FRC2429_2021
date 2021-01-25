
from wpilib.command import Command
from wpilib.trajectory.constraint import DifferentialDriveVoltageConstraint
import wpilib.trajectory
import wpilib.controller
import wpilib.kinematics
import wpilib.geometry as geo
from wpilib import Timer, SmartDashboard
import subsystems.drive_constants as drive_constants

class AutonomousRamsete(Command):
    """Attempting to translate the Ramsete command from commands V2 into a V1 version since robotpy doesn't have this command yet"""
    def __init__(self, robot, timeout=10):
        Command.__init__(self, name='autonomous_ramsete')
        self.robot = robot
        self.requires(robot.drivetrain)
        self.setTimeout(timeout)
        self.previous_time = -1
        self.use_PID = True

        # Create a voltage constraint to ensure we don't accelerate too fast
        self.feed_forward = wpilib.controller.SimpleMotorFeedforwardMeters(
            drive_constants.ks_volts, drive_constants.kv_volt_seconds_per_meter, drive_constants.ka_volt_seconds_squared_per_meter)
        self.autonomous_voltage_constraint = DifferentialDriveVoltageConstraint(self.feed_forward, drive_constants.drive_kinematics, 10)

        # create controllers
        self.follower = wpilib.controller.RamseteController(drive_constants.ramsete_B, drive_constants.ramsete_Zeta)
        self.left_controller = wpilib.controller.PIDController(drive_constants.kp_drive_vel, 0 , 0)
        self.right_controller = wpilib.controller.PIDController(drive_constants.kp_drive_vel, 0 , 0)


        # Create config for trajectory
        self.config = wpilib.trajectory.TrajectoryConfig(
            drive_constants.k_max_speed_meters_per_second, drive_constants.k_max_acceleration_meters_per_second_squared)
        self.config.setKinematics(drive_constants.drive_kinematics)
        self.config.addConstraint(self.autonomous_voltage_constraint)

        # example trajectory to test
        self.start_pose = geo.Pose2d(0, 0, geo.Rotation2d(0))
        self.end_pose = geo.Pose2d(6, 0, geo.Rotation2d(0))
        self.midpoints = [geo.Translation2d(3, 0), geo.Translation2d(3, 0)]
        self.trajectory = wpilib.trajectory.TrajectoryGenerator.generateTrajectory(self.start_pose, self.midpoints, self.end_pose, self.config)
        #example_trajectory = wpilib.trajectory.TrajectoryGenerator.generateTrajectory(start_pose, midpoints, end_pose, config)
        # kinematics
        self.kinematics = drive_constants.drive_kinematics

    # alternately, import a pathweaver json
    # https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)

        self.robot.drivetrain.reset_odometry(self.start_pose)

        self.previous_time = -1
        initial_state = self.trajectory.sample(0)
        # these are all meters in 2021
        self.previous_speeds = self.kinematics.toWheelSpeeds(wpilib.kinematics.ChassisSpeeds(
            initial_state.velocity, 0, initial_state.curvature*initial_state.velocity))
        self.left_controller.reset()
        self.right_controller.reset()

    def execute(self) -> None:
        current_time = self.timeSinceInitialized()
        dt = current_time - self.previous_time

        if self.previous_time < 0:
            self.robot.drivetrain.tank_drive_volts(0, 0)
            self.previous_time = current_time
            return

        target_wheel_speeds = self.kinematics.toWheelSpeeds(
            self.follower.calculate(self.robot.drivetrain.get_pose(), self.trajectory.sample(current_time)))
        #speed_limit = 5
        #left_speed_setpoint = max(-speed_limit, min(speed_limit, target_wheel_speeds.left))
        #right_speed_setpoint = max(-speed_limit, min(speed_limit, target_wheel_speeds.left))
        left_speed_setpoint = target_wheel_speeds.left
        right_speed_setpoint = target_wheel_speeds.left
        if self.use_PID:
            left_feed_forward = self.feed_forward.calculate(left_speed_setpoint, (left_speed_setpoint - self.previous_speeds.left)/dt)
            right_feed_forward = self.feed_forward.calculate(right_speed_setpoint, (right_speed_setpoint - self.previous_speeds.right)/dt)
            #ws_left, ws_right = self.robot.drivetrain.get_wheel_speeds().left, self.robot.drivetrain.get_wheel_speeds().right
            ws_left, ws_right = self.robot.drivetrain.l_encoder.getRate(), self.robot.drivetrain.r_encoder.getRate()
            left_output = left_feed_forward + self.left_controller.calculate(ws_left, left_speed_setpoint)
            right_output = right_feed_forward + self.left_controller.calculate(ws_right, right_speed_setpoint)
        else:
            left_output = left_speed_setpoint
            right_output = right_speed_setpoint

        #left_output, right_output = 5, -5
        self.robot.drivetrain.tank_drive_volts(left_output + 0, right_output -0)
        self.previous_speeds = target_wheel_speeds
        self.previous_time = current_time
        self.robot.drivetrain.drive.feed()
        SmartDashboard.putNumber('left_speed_setpoint', left_speed_setpoint)
        SmartDashboard.putNumber('right_speed_setpoint', right_speed_setpoint)
        SmartDashboard.putNumber('left_feed_forward', left_feed_forward)
        SmartDashboard.putNumber('right_feed_forward', right_feed_forward)


    def isFinished(self) -> bool:
        return self.isTimedOut() or self.timeSinceInitialized() > self.trajectory.totalTime()

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** {message} {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        self.robot.drivetrain.stop()

    def interrupted(self):
        self.end(message='Interrupted')
