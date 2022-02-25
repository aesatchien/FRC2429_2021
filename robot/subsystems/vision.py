from wpilib.command import Subsystem
import networktables
from wpilib import SmartDashboard
from networktables import NetworkTables
from wpilib import DriverStation

class Vision(Subsystem):
    def __init__(self) -> None:
        super().__init__('vision')
        self.counter = 0
        
        self.ballcam_table = NetworkTables.getTable('BallCam')
        self.fms_info_table = NetworkTables.getTable('FMSInfo')
        self.camera_dict = {'red': {}, 'blue': {}}

        for key in self.camera_dict.keys():
            self.camera_dict[key].update({'targets_entry': self.ballcam_table.getEntry(f"/{key}/targets")})
            self.camera_dict[key].update({'distance_entry': self.ballcam_table.getEntry(f"/{key}/distance")})
            self.camera_dict[key].update({'rotation_entry': self.ballcam_table.getEntry(f"/{key}/rotation")})

        self.targets = 0
        self.distance = 0
        self.rotation = 0
        
        self.team_color = 'red'

    def periodic(self) -> None:
        self.counter += 1

        # update five times a second
        if self.counter % 10 == 0:
            # Use DriverStation.getAlliance()
            if (self.fms_info_table.getEntry('IsRedAlliance').getBoolean(True)):
                self.team_color = 'red'
            else:
                self.team_color = 'blue'

            self.targets = self.camera_dict[self.team_color]['targets_entry'].getDouble(0)
            self.distance = self.camera_dict[self.team_color]['distance_entry'].getDouble(0)
            self.rotation = self.camera_dict[self.team_color]['rotation_entry'].getDouble(0)

            SmartDashboard.putNumber('siraaj', 7.0)
            SmartDashboard.putNumber('ball_targets', self.targets)
            SmartDashboard.putNumber('ball_distance', self.distance)
            SmartDashboard.putNumber('ball_rotation', self.rotation)

    def getBallValues(self):
        return (self.targets > 0, self.rotation, self.distance)