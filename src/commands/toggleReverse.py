import commands2
import wpilib
from subSystems.REVDriveSubsystem import DriveSubsystem

import constants

class ToggleReverse(commands2.Command):
    def __init__(self, drive: DriveSubsystem):
        super().__init__()
        self.dr = drive

    def initialize(self):
        self.dr.toggleReverse()

    def execute(self):
        pass
        
    def isFinished(self) -> bool:
        return True