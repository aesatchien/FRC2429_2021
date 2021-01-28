
from wpilib.command import Command
from wpilib.trajectory.constraint import DifferentialDriveVoltageConstraint
import wpilib.trajectory
import wpilib.controller
import wpilib.kinematics
import wpilib.geometry as geo
from wpilib import Timer, SmartDashboard
import subsystems.drive_constants as drive_constants

class AutonomousRamsete(Command):
    """Attempting to translate the Ramsete command from commands V2 into a V1 version since robotpy doesn't have this command yet
    this does not work yet - it just sits there and quivers.  Have to go through piece by piece """
    def __init__(self, robot, timeout=30):
        Command.__init__(self, name='auto_ramsete')
        self.robot = robot
        self.requires(robot.drivetrain)
        self.setTimeout(timeout)
        self.previous_time = -1
        self.previous_speeds = None
        self.use_PID = True
        self.counter = 0

        # Create a voltage constraint to ensure we don't accelerate too fast
        self.feed_forward = wpilib.controller.SimpleMotorFeedforwardMeters(
            drive_constants.ks_volts, drive_constants.kv_volt_seconds_per_meter, drive_constants.ka_volt_seconds_squared_per_meter)
        self.autonomous_voltage_constraint = DifferentialDriveVoltageConstraint(self.feed_forward, drive_constants.drive_kinematics, 10)

        # create controllers
        self.follower = wpilib.controller.RamseteController(drive_constants.ramsete_B, drive_constants.ramsete_Zeta)
        self.kp_vel = drive_constants.kp_drive_vel; SmartDashboard.putNumber("kp_vel_ramsete", self.kp_vel)
        self.left_controller = wpilib.controller.PIDController(self.kp_vel, 0 , 0)
        self.right_controller = wpilib.controller.PIDController(self.kp_vel, 0 , 0)


        # Create config for trajectory
        self.config = wpilib.trajectory.TrajectoryConfig(
            drive_constants.k_max_speed_meters_per_second, drive_constants.k_max_acceleration_meters_per_second_squared)
        self.config.setKinematics(drive_constants.drive_kinematics)
        self.config.addConstraint(self.autonomous_voltage_constraint)

        # example trajectory to test
        self.start_pose = geo.Pose2d(0, 0, geo.Rotation2d(0))
        self.end_pose = geo.Pose2d(3, 0, geo.Rotation2d(0))
        self.midpoints = [geo.Translation2d(1, 1), geo.Translation2d(2, -1)]
        test_trajectory = wpilib.trajectory.TrajectoryGenerator.generateTrajectory(self.start_pose, self.midpoints, self.end_pose, self.config)

        # minimum slalom test - nice thing about having the points means you can change the speeds for the
        # trajectory config and then you can go faster and faster.  but it's better to use the pathweaver once you have the speeds you want
        self.start_pose = geo.Pose2d(0, 0, geo.Rotation2d(0))
        self.end_pose = geo.Pose2d(0, 1.7, geo.Rotation2d(3.14))
        self.midpoints = [geo.Translation2d(0.97, 0.23), geo.Translation2d(1.93, 1.59), geo.Translation2d(3.47, 2.05),
                          geo.Translation2d(5.38, 1.18), geo.Translation2d(6.48, -0.01), geo.Translation2d(7.36, 0.88),
                          geo.Translation2d(6.38, 1.76), geo.Translation2d(5.53, 0.85), geo.Translation2d(5.10, 0.20),
                          geo.Translation2d(3.55, -0.05), geo.Translation2d(1.84, 0.16), geo.Translation2d(1.21, 0.84),
                          geo.Translation2d(0.51, 1.55)]

        self.trajectory = wpilib.trajectory.TrajectoryGenerator.generateTrajectory(self.start_pose, self.midpoints, self.end_pose, self.config)

        self.kinematics = drive_constants.drive_kinematics

    # alternately, import a pathweaver json
    # https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time} s **")

        self.robot.drivetrain.reset_odometry(self.start_pose)

        self.previous_time = -1
        initial_state = self.trajectory.sample(0)
        # these are all meters in 2021
        self.previous_speeds = self.kinematics.toWheelSpeeds(wpilib.kinematics.ChassisSpeeds(
            initial_state.velocity, 0, initial_state.curvature*initial_state.velocity))

        self.kp_vel = SmartDashboard.getNumber("kp_vel_ramsete", self.kp_vel)
        self.left_controller = wpilib.controller.PIDController(self.kp_vel, 0 , 0)
        self.right_controller = wpilib.controller.PIDController(self.kp_vel, 0 , 0)
        self.left_controller.reset()
        self.right_controller.reset()
        print('Time\tTr Vel\tTr Rot\tlspd\trspd\tram ang\tram vx\tram vy\tlffw\trffw\tlpid\trpid')

    def execute(self) -> None:
        self.counter += 1
        current_time = self.timeSinceInitialized()
        dt = current_time - self.previous_time

        if self.previous_time < 0:
            self.robot.drivetrain.tank_drive_volts(0, 0)
            self.previous_time = current_time
            return

        ramsete = self.follower.calculate(self.robot.drivetrain.get_pose(), self.trajectory.sample(current_time))
        target_wheel_speeds = self.kinematics.toWheelSpeeds(ramsete)

        left_speed_setpoint = target_wheel_speeds.left
        right_speed_setpoint = target_wheel_speeds.right
        if self.use_PID:
            left_feed_forward = self.feed_forward.calculate(left_speed_setpoint, (left_speed_setpoint - self.previous_speeds.left)/dt)
            right_feed_forward = self.feed_forward.calculate(right_speed_setpoint, (right_speed_setpoint - self.previous_speeds.right)/dt)
            #ws_left, ws_right = self.robot.drivetrain.get_wheel_speeds().left, self.robot.drivetrain.get_wheel_speeds().right
            ws_left, ws_right = self.robot.drivetrain.l_encoder.getRate(), self.robot.drivetrain.r_encoder.getRate()
            left_output_pid = self.left_controller.calculate(ws_left, left_speed_setpoint)
            right_output_pid = self.right_controller.calculate(ws_right, right_speed_setpoint)
            left_output = -left_output_pid + left_feed_forward
            right_output = -right_output_pid + right_feed_forward

        else:
            left_output = left_speed_setpoint
            right_output = right_speed_setpoint

        self.robot.drivetrain.tank_drive_volts(left_output,  -right_output)
        self.previous_speeds = target_wheel_speeds
        self.previous_time = current_time
        self.robot.drivetrain.drive.feed()

        SmartDashboard.putNumber('left_speed_setpoint', left_speed_setpoint)
        SmartDashboard.putNumber('right_speed_setpoint', right_speed_setpoint)

        if self.counter % 10 == 0:
            out_string = f'{current_time:2.2f}\t{self.trajectory.sample(current_time).velocity:2.1f}\t{self.trajectory.sample(current_time).pose.rotation().radians():2.2f}\t'
            out_string += f'{left_speed_setpoint:2.2f}\t{right_speed_setpoint:2.2f}\t{ramsete.omega:2.2f}\t{ramsete.vx:2.2f}\t{ramsete.vy:2.2f}\t'
            out_string += f'{left_feed_forward:2.2f}\t{right_feed_forward:2.2f}\t{left_output_pid:2.2f}\t{right_output_pid:2.2f}'
            print(out_string)

    def isFinished(self) -> bool:
        return self.isTimedOut() or self.timeSinceInitialized() > self.trajectory.totalTime()

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print(f"** {message} {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        self.robot.drivetrain.stop()

    def interrupted(self):
        self.end(message='Interrupted')
