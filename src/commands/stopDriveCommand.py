from commands2 import Command
from subSystems.REVDriveSubsystem import DriveSubsystem
import constants
import wpilib

class StopDriveCommand(Command):
    def __init__(self, driveSubsystem: DriveSubsystem):
        super().__init__()
        self.driveSubsystem = driveSubsystem
        self.addRequirements(self.driveSubsystem)

    def execute(self):
        self.driveSubsystem.drive(0, 0)
    
    def isFinished(self):
        return True  # Makes the command run only once per scheduler call
