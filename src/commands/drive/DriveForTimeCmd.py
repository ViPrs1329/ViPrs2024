from commands2 import Command
import wpilib
from subSystems.REVDriveSubsystem import DriveSubsystem

class DriveForTime(Command):
    def __init__(self, drive: DriveSubsystem, speed: float, time: float):
        super().__init__()
        self.drive = drive
        self.speed = speed
        self.time = time
        self.timer = wpilib.Timer()
        self.addRequirements(drive)
        print("DriveForTime.__init__(..., speed={speed}, time={time})")

    def initialize(self):
        self.timer.reset()
        self.timer.start()

    def execute(self):
        self.drive.arcadeDriveSS(self.speed, 0.0)
        print("DriveForTime.execute() - Driving...")

    def end(self, interrupted):
        self.drive.arcadeDriveSS(0.0, 0.0)
        self.timer.stop()
        print("DriveForTime.end() - Driving has ended")


    def isFinished(self):
        return self.timer.hasElapsed(self.time)
