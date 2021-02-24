# shooter to use both in sim and robot mode - sim handles the Sparkmax w/o crashing now but does not simulate well

from wpilib.command import Subsystem
import wpilib.controller
from wpilib import Spark, Encoder, DigitalInput
import rev

class Shooter(Subsystem):
    # ----------------- INITIALIZATION -----------------------
    def __init__(self, robot):
        super().__init__("shooter")
        self.robot = robot
        self.counter = 0  # used for updating the log
        self.feed_forward = 5.5  # default volts to give the flywheel to get close to setpoint, optional

        # motor controllers
        self.sparkmax_flywheel = rev.CANSparkMax(5, rev.MotorType.kBrushless)
        self.spark_hood = Spark(5)
        self.spark_feed = Spark(6)

        # encoders and PID controllers
        self.hood_encoder = Encoder(4, 5)  # generic encoder - we'll have to install one on the 775 motor
        self.hood_encoder.setDistancePerPulse(1/1024)
        self.hood_controller = wpilib.controller.PIDController(Kp=0.1, Ki=0, Kd=0.0)
        self.hood_setpoint = 45
        self.flywheel_encoder = self.sparkmax_flywheel.getEncoder()  # built-in to the sparkmax/neo
        self.flywheel_controller = self.sparkmax_flywheel.getPIDController()  # built-in PID controller in the sparkmax


        # limit switches, use is TBD
        self.limit_low = DigitalInput(6)
        self.limit_high = DigitalInput(7)

    def set_flywheel(self, velocity):
        self.flywheel_controller.setReference(velocity, rev.ControlType.kVelocity, 0, self.feed_forward)

    def stop_flywheel(self):
        self.flywheel_controller.setReference(0, rev.ControlType.kVoltage)

    def set_feed_motor(self, speed):
        self.spark_feed.set(speed)

    def stop_feed_motor(self):
        self.spark_feed.set(0)

    def set_hood_motor(self, power):
        self.spark_hood.set(power)

    def change_elevation(self, power):  # open loop approach
        if power > 0 and not self.limit_high.get():
            self.set_hood_motor(power)
        elif power < 0 and not self.limit_low.get():
            self.set_hood_motor(power)
        else:
            self.set_hood_motor(0)

    def set_hood_setpoint(self, setpoint):
        self.hood_controller.setSetpoint(setpoint)

    def get_angle(self):
        elevation_minimum = 30  # degrees
        conversion_factor = 1  # encoder units to degrees
        # return elevation_minimum + conversion_factor * self.hood_encoder.getDistance()
        return self.hood_encoder.getDistance()

    def periodic(self) -> None:
        """Perform necessary periodic updates"""
        self.counter += 1

        if self.counter % 5 == 0:
            pass
            # ten per second updates
            #self.error = self.get_angle() - self.hood_setpoint
            #output = self.hood_controller.calculate(self.error)
            #self.change_elevation(output)
        if self.counter % 50 == 0:
            pass
            #  print(f'{self.error} {self.hood_setpoint} {self.get_angle()}')