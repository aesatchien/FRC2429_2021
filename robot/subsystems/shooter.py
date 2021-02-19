#Shooter subsystem - flywheel, angle adjuster, feed motor
import wpilib
from wpilib.command import Subsystem
from wpilib import Encoder, SmartDashboard, Spark
import rev

class Shooter(Subsystem):
    def __init__(self, robot):
        Subsystem.__init__(self, "Shooter")
        self.robot = robot

        self.sparkmax_flywheel = rev.CANSparkMax(5, rev.MotorType.kBrushless)
        self.flywheel_encoder = rev.CANSparkMax.getEncoder(self.sparkmax_flywheel)
        
        #775 motors
        self.spark_hood = Spark(1)
        self.spark_feed = Spark(2)

        #Not sure what to put for the encoder values
        self.hood_encoder = Encoder(1,2)

        self.limit_hi = 1
        self.limit_lo = 0
        
        self.enabled = False

    def get_velocity(self):
        return self.flywheel_encoder.getVelocity()

    def is_enabled(self, enabled):
        if(enabled):
            self.enabled = True

    def set_flywheel(self, power):
        self.sparkmax_flywheel.set(power)  

    def stop_flywheel(self):
        self.sparkmax_flywheel.set(0)
        self.enabled = False


    def set_feed_motor(self, speed):
        if(self.enabled):
            self.spark_feed.set(speed)
        else:
            print("Flywheel not at correct speed. Current Speed: " + self.flywheel_encoder.getVelocity())


    def is_enabled(self):
        return self.enabled


    def stop_feed_motor(self):
        self.spark_feed.set(0)


    def get_angle(self):
        return self.hood_encoder.getDistance()

    def set_elevation(self, power):
        self.spark_hood.set(power)
        
        if(self.limit_lo < self.hood_encoder.getDirection < self.limit_hi):
            self.spark_hood.set(power)
        elif(self.limit_lo > self.hood_encoder.getDirection):
            self.spark_hood.set(self.limit_lo)
        elif(self.limit_hi < self.hood_encoder.getDirection):
            self.spark_hood.set(self.limit_hi)


    def reset_elevation(self):
        self.spark_hood.set(0)
        self.hood_encoder.reset()
            