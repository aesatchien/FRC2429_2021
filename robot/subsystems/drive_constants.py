import math
import wpilib.kinematics

# drivetrain constants
wheel_diameter_in = 8  # wheel diameter in inches
wheel_diameter_m = 8 * 0.0254  # wheel diameter in inches

# Configure encoders and controllers
# should be wheel_diameter * pi / gear_ratio - and for the old double reduction gear box the gear ratio was 4.17:1.
# With the shifter (low gear) I think it was a 12.26.  Then new 2020 WCD gearbox is 9.52, and the tuffbox is 12.75
gear_ratio = 9.52
sparkmax_conversion_factor_inches = wheel_diameter_in * math.pi / gear_ratio
sparkmax_conversion_factor_meters = wheel_diameter_m * math.pi / gear_ratio

# pretend encoders for simulation
encoder_CPR = 1024 # encoder counts per revolution
# encoder_distance_per_pulse_m = wheel_diameter_m * math.pi / (encoder_CPR * gear_ratio)
encoder_distance_per_pulse_m = wheel_diameter_m * math.pi / (encoder_CPR )

# set up the wpilib kinematics model
track_width_meters = 0.69
drive_kinematics = wpilib.kinematics.DifferentialDriveKinematics(track_width_meters)

# get these from robot characterization tools - using example values for now
# ToDo: characterize this on the real robot
ks_volts = 0.1
kv_volt_seconds_per_meter = 0.1
ka_volt_seconds_squared_per_meter = 0.1
kp_drive_vel = 1

# constants for autonomous
k_max_speed_meters_per_second = 1
k_max_acceleration_meters_per_second_squared = 0.5

# Reasonable baseline values for a RAMSETE follower in units of meters and seconds
ramsete_B = 2
ramsete_Zeta = 0.7