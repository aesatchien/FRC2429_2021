# intended to have the robot follow a trajectory path
# ToDo: make sure velocity controllers are actually helping and optimize the B, Z and kp gains
from wpilib.command import Command
import wpilib.controller
import wpilib.kinematics
from wpilib import Timer, SmartDashboard
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
import wpimath.trajectory
import wpimath.geometry as geo

from pathlib import Path
from datetime import datetime
import pickle

import subsystems.drive_constants as drive_constants

class AutonomousRamsete(Command):
    """Attempting to translate the Ramsete command from commands V2 into a V1 version since robotpy doesn't have this command yet
    this does not work yet - it just sits there and quivers.  Have to go through piece by piece """
    def __init__(self, robot, timeout=50):
        Command.__init__(self, name='auto_ramsete')
        self.robot = robot
        self.requires(robot.drivetrain)
        self.setTimeout(timeout)
        self.previous_time = -1
        self.previous_speeds = None
        self.use_PID = True
        self.counter = 0
        self.telemetry = []
        self.trajectory = None
        self.write_telemetry = False
        self.dash = True

        # constants for ramsete follower and velocity PID controllers
        self.beta = drive_constants.ramsete_B
        self.zeta = drive_constants.ramsete_Zeta
        self.kp_vel = drive_constants.kp_drive_vel
        self.kd_vel = 0

        if self.dash is True:
            SmartDashboard.putNumber("ramsete_kpvel", self.kp_vel)
            SmartDashboard.putNumber("ramsete_B", self.beta)
            SmartDashboard.putNumber("ramsete_Z", self.zeta)
            SmartDashboard.putBoolean("ramsete_write", self.write_telemetry)

        self.feed_forward = drive_constants.feed_forward
        self.kinematics = drive_constants.drive_kinematics
        self.course = drive_constants.course

    def initialize(self):
        """Called just before this Command runs the first time."""

        self.previous_time = -1
        self.telemetry = []

        # update gains from dash if desired
        if self.dash is True:
            self.kp_vel = SmartDashboard.getNumber("ramsete_kpvel", self.kp_vel)
            self.beta = SmartDashboard.getNumber("ramsete_B", self.beta)
            self.zeta = SmartDashboard.getNumber("ramsete_Z", self.zeta)

        # create controllers
        self.follower = wpilib.controller.RamseteController(self.beta, self.zeta)
        self.left_controller = wpilib.controller.PIDController(self.kp_vel, 0 , self.kd_vel)
        self.right_controller = wpilib.controller.PIDController(self.kp_vel, 0 , self.kd_vel)
        self.left_controller.reset()
        self.right_controller.reset()


        #ToDo - make this selectable, probably from the dash, add the other trajectories

        # Note - we are setting to pose to have the robot physically in the start position - usually absolute matters
        trajectory_choice = self.robot.oi.path_chooser.getSelected()  # get path from the GUI
        if 'slalom' in trajectory_choice and 'pw' in trajectory_choice:
            self.course = trajectory_choice
            self.trajectory = drive_constants.get_pathweaver_trajectory(trajectory_choice)
            self.start_pose = geo.Pose2d(1.3, 0.66, geo.Rotation2d(0))
        elif 'barrel' in trajectory_choice and 'pw' in trajectory_choice:
            self.course = trajectory_choice
            self.trajectory = drive_constants.get_pathweaver_trajectory(trajectory_choice)
            self.start_pose = geo.Pose2d(1.3, 2.40, geo.Rotation2d(0))
        elif 'bounce' in trajectory_choice and 'pw' in trajectory_choice:
            self.course = trajectory_choice
            self.trajectory = drive_constants.get_pathweaver_trajectory(trajectory_choice)
            self.start_pose = geo.Pose2d(1.3, 2.62, geo.Rotation2d(0))
        elif trajectory_choice == 'loop':
            self.course = 'loop'
            self.trajectory = drive_constants.get_loop_trajectory()
            self.start_pose = geo.Pose2d(0, 0, geo.Rotation2d(0))
        elif trajectory_choice == 'poses':
            self.course = 'slalom_poses'
            self.trajectory = drive_constants.get_pose_trajectory()
            self.start_pose = geo.Pose2d(0, 0, geo.Rotation2d(0))
        elif trajectory_choice == 'points':
            self.course = 'slalom_points'
            self.trajectory = drive_constants.get_point_trajectory()
            self.start_pose = geo.Pose2d(0, 0, geo.Rotation2d(0))
        else:
            self.course = 'test'
            self.trajectory = drive_constants.get_test_trajectory()
            self.start_pose = geo.Pose2d(0, 0, geo.Rotation2d(0))
        #SmartDashboard.set ('obstacles', self.course)

        self.robot.drivetrain.reset_odometry(self.start_pose)  # ToDo need to sort this out - pathweaver vs. self-made padding
        initial_state = self.trajectory.sample(0)
        # these are all meters in 2021
        self.previous_speeds = self.kinematics.toWheelSpeeds(wpilib.kinematics.ChassisSpeeds(
            initial_state.velocity, 0, initial_state.curvature*initial_state.velocity))

        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} on {self.course} (b={self.beta}, z={self.zeta}, kp_vel={self.kp_vel}) at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time} s **")

        print('Time\tTr Vel\tTr Rot\tlspd\trspd\tram ang\tram vx\tram vy\tlffw\trffw\tlpid\trpid')

    def execute(self) -> None:
        self.counter += 1
        current_time = self.timeSinceInitialized()
        dt = current_time - self.previous_time

        if self.previous_time < 0:
            self.robot.drivetrain.tank_drive_volts(0, 0)
            self.previous_time = current_time
            return

        # get the robot's current field pose, current trajectory point, and feed to the ramsete controller
        pose = self.robot.drivetrain.get_pose()
        sample = self.trajectory.sample(current_time)
        ramsete = self.follower.calculate(pose, sample)
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
            # 100% sure that these signs are right - see plots.   Want minus the PID and plus the feedfwd
            left_output = -left_output_pid + left_feed_forward
            right_output = -right_output_pid + right_feed_forward

        else:
            left_output = left_speed_setpoint
            right_output = right_speed_setpoint

        self.robot.drivetrain.tank_drive_volts(left_output,  -right_output)
        self.previous_speeds = target_wheel_speeds
        self.previous_time = current_time
        self.robot.drivetrain.drive.feed()

        #SmartDashboard.putNumber('rz_left_speed_setpoint', left_speed_setpoint)
        #SmartDashboard.putNumber('rz_right_speed_setpoint', right_speed_setpoint)

        if self.counter % 5 == 0:  # ten times per second
            telemetry_data = {'TIME':current_time, 'RBT_X':pose.X(), 'RBT_Y':pose.Y(), 'RBT_TH':pose.rotation().radians(),
                            'RBT_VEL':self.robot.drivetrain.get_average_encoder_rate(),
                            'RBT_RVEL':self.robot.drivetrain.r_encoder.getRate(), 'RBT_LVEL':self.robot.drivetrain.l_encoder.getRate(),
                            'TRAJ_X':sample.pose.X(), 'TRAJ_Y':sample.pose.Y(), 'TRAJ_TH':sample.pose.rotation().radians(), 'TRAJ_VEL':sample.velocity,
                            'RAM_VELX':ramsete.vx, 'RAM_LVEL_SP':left_speed_setpoint, 'RAM_RVEL_SP':right_speed_setpoint,
                            'RAM_OM':ramsete.omega, 'LFF':left_feed_forward, 'RFF':right_feed_forward, 'LPID': left_output_pid, 'RPID':right_output_pid}
            self.telemetry.append(telemetry_data)
        if self.counter % 20 == 0:
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

        self.write_telemetry = SmartDashboard.getBoolean("ramsete_write", self.write_telemetry)
        if self.write_telemetry:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            file_name = timestamp + '_'+ self.course + f'_kpvel_{self.kp_vel:2.1f}'.replace('.','p') + f'vel_{drive_constants.k_max_speed_meters_per_second}'   +'.pkl'
            pickle_file = Path.cwd() / 'sim' / 'data' / file_name
            with open(pickle_file.absolute(), 'wb') as fp:
                out_dict = {'TIMESTAMP':timestamp,'DATA':self.telemetry, 'COURSE':self.course, 'VELOCITY':drive_constants.k_max_speed_meters_per_second,
                            'KP_VEL':self.kp_vel, 'KD_VEL':self.kd_vel, 'BETA':self.beta, 'ZETA':self.zeta}
                pickle.dump(out_dict, fp)
            print(f'*** Wrote ramsete command data to {file_name} ***')
        else:
            print(f'*** Skipping saving of telemetry to disk ***')

    def interrupted(self):
        self.end(message='Interrupted')
