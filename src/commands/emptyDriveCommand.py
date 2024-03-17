import commands2
from subSystems.REVDriveSubsystem import DriveSubsystem
import constants
import wpilib

class emptyDriveCommand(commands2.Command):
    def __init__(self, driveSubsystem: DriveSubsystem):
        super().__init__()
        self.driveSubsystem = driveSubsystem

    def execute(self):
        self.driveSubsystem.robotDrive.arcadeDrive(0, 0, False)

    def isFinished(self):
        return False

    def end(self, interrupted):
        pass