# intended to have the robot follow a trajectory path
# ToDo: make sure velocity controllers are actually helping and optimize the B, Z and kp gains
from wpilib.command import Command
import wpilib.controller
import wpimath.kinematics
from wpilib import Timer, SmartDashboard
import wpimath.geometry as geo

from pathlib import Path
from datetime import datetime
import pickle

import subsystems.drive_constants as drive_constants

class AutonomousRamseteSimple(Command):
    """Attempting to translate the Ramsete command from commands V2 into a V1 version since robotpy doesn't have this command yet """

    # constants for ramsete follower and velocity PID controllers - don't want a different copy for each command
    beta = drive_constants.ramsete_B
    zeta = drive_constants.ramsete_Zeta
    kp_vel = drive_constants.kp_drive_vel
    kd_vel = 0
    velocity = drive_constants.k_max_speed_meters_per_second
    write_telemetry = False

    def __init__(self, robot, path:str, relative=False, reset_telemetry=True, timeout=50):
        Command.__init__(self, name='auto_ramsete_simple')
        self.robot = robot
        self.requires(robot.drivetrain)
        self.setTimeout(timeout)

        self.previous_time = -1
        self.previous_speeds = None
        self.use_PID = True
        self.counter = 0
        self.telemetry = []
        self.trajectory = None

        self.relative = relative
        self.path = path
        self.reset_telemetry = reset_telemetry
        self.dash = True

        self.feed_forward = drive_constants.feed_forward
        self.kinematics = drive_constants.drive_kinematics
        self.course = drive_constants.course

    def initialize(self):
        """Called just before this Command runs the first time."""

        self.previous_time = -1
        if self.reset_telemetry:  # don't reset telemetry if we want to chain trajectories together
            self.telemetry = []
        self.init_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)

        self.robot.drivetrain.drive.feed() # this initialization is taking some time now
        # update gains from dash if desired
        if self.dash is True:
            self.kp_vel = SmartDashboard.getNumber("ramsete_kpvel", self.kp_vel)
            self.beta = SmartDashboard.getNumber("ramsete_B", self.beta)
            self.zeta = SmartDashboard.getNumber("ramsete_Z", self.zeta)
            self.write_telemetry = SmartDashboard.getBoolean("ramsete_write", self.write_telemetry)

        # create controllers
        self.follower = wpilib.controller.RamseteController(self.beta, self.zeta)
        self.left_controller = wpilib.controller.PIDController(self.kp_vel, 0 , self.kd_vel)
        self.right_controller = wpilib.controller.PIDController(self.kp_vel, 0 , self.kd_vel)
        self.left_controller.reset()
        self.right_controller.reset()


        #ToDo - make this selectable, probably from the dash, add the other trajectories (done)
        trajectory_choice = self.path  # path is passed in on constructor this time
        self.velocity = float(self.robot.oi.velocity_chooser.getSelected())  # get the velocity from the GUI
        self.trajectory = drive_constants.generate_trajectory(trajectory_choice, self.velocity, save=False)
        self.course = trajectory_choice
        # Note - we are setting to pose to have the robot physically in the start position - usually absolute matters
        if self.relative:  # figure out if we want to start where we are or where the trajectory says we should be
            self.start_pose = geo.Pose2d(self.trajectory.sample(0).pose.X(),self.trajectory.sample(0).pose.Y(),
                                         self.robot.drivetrain.get_rotation2d())
        else:
            self.start_pose = self.robot.drivetrain.get_pose()
        self.robot.drivetrain.reset_odometry(self.start_pose)

        self.robot.drivetrain.drive.feed()  # this initialization is taking some time now

        initial_state = self.trajectory.sample(0)
        # these are all meters in 2021
        self.previous_speeds = self.kinematics.toWheelSpeeds(wpilib.kinematics.ChassisSpeeds(
            initial_state.velocity, 0, initial_state.curvature*initial_state.velocity))

        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} on {self.course} with load time {self.start_time-self.init_time:2.2f}s"
                     f" (b={self.beta}, z={self.zeta}, kp_vel={self.kp_vel}) at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time} s **")

        print('Time\tTr Vel\tTr Rot\tlspd\trspd\tram ang\tram vx\tram vy\tlffw\trffw\tlpid\trpid')

    def execute(self) -> None:

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

        else:  # ToDo - fix this to just be the feed forwards and test it
            left_output = left_speed_setpoint
            right_output = right_speed_setpoint

        self.robot.drivetrain.tank_drive_volts(left_output, -right_output)
        self.previous_speeds = target_wheel_speeds
        self.previous_time = current_time
        self.robot.drivetrain.drive.feed()  # should this be in tank drive?

        if self.counter % 5 == 0:  # ten times per second update the telemetry array
            telemetry_data = {'TIME':current_time, 'RBT_X':pose.X(), 'RBT_Y':pose.Y(), 'RBT_TH':pose.rotation().radians(),
                            'RBT_VEL':self.robot.drivetrain.get_average_encoder_rate(),
                            'RBT_RVEL':self.robot.drivetrain.r_encoder.getRate(), 'RBT_LVEL':self.robot.drivetrain.l_encoder.getRate(),
                            'TRAJ_X':sample.pose.X(), 'TRAJ_Y':sample.pose.Y(), 'TRAJ_TH':sample.pose.rotation().radians(), 'TRAJ_VEL':sample.velocity,
                            'RAM_VELX':ramsete.vx, 'RAM_LVEL_SP':left_speed_setpoint, 'RAM_RVEL_SP':right_speed_setpoint,
                            'RAM_OM':ramsete.omega, 'LFF':left_feed_forward, 'RFF':right_feed_forward, 'LPID': left_output_pid, 'RPID':right_output_pid}
            self.telemetry.append(telemetry_data)
        if self.counter % 50 == 0: # once per second send data to the console
            out_string = f'{current_time:2.2f}\t{self.trajectory.sample(current_time).velocity:2.1f}\t{self.trajectory.sample(current_time).pose.rotation().radians():2.2f}\t'
            out_string += f'{left_speed_setpoint:2.2f}\t{right_speed_setpoint:2.2f}\t{ramsete.omega:2.2f}\t{ramsete.vx:2.2f}\t{ramsete.vy:2.2f}\t'
            out_string += f'{left_feed_forward:2.2f}\t{right_feed_forward:2.2f}\t{left_output_pid:2.2f}\t{right_output_pid:2.2f}'
            print(out_string)
        self.counter += 1

    def isFinished(self) -> bool:  # ToDo: investigate the different end methods
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
            file_name = timestamp + '_'+ self.course + f'_kpv_{self.kp_vel:2.1f}'.replace('.','p') + f'_vel_{str(self.velocity).replace(".","p")}'   +'.pkl'
            pickle_file = Path.cwd() / 'sim' / 'data' / file_name
            with open(pickle_file.absolute(), 'wb') as fp:
                out_dict = {'TIMESTAMP':timestamp,'DATA':self.telemetry, 'COURSE':self.course, 'VELOCITY':self.velocity,
                            'KP_VEL':self.kp_vel, 'KD_VEL':self.kd_vel, 'BETA':self.beta, 'ZETA':self.zeta}
                pickle.dump(out_dict, fp)
            print(f'*** Wrote ramsete command data to {file_name} ***')
        else:
            print(f'*** Skipping saving of telemetry to disk ***')

    def interrupted(self):
        self.end(message='Interrupted')
