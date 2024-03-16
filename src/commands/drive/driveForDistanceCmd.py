from commands2 import Command
from subSystems.REVDriveSubsystem import DriveSubsystem

class DriveForDistanceCmd(Command):
    def __init__(self, drive: DriveSubsystem, distance: float):
        super().__init__()
        self.drive = drive
        self.distance = distance
        # Reset encoder positions to zero at the start
        # self.drive.leftFrontEncoder.setPosition(0)
        # self.drive.rightFrontEncoder.setPosition(0)
        self.addRequirements(drive)
        print(f"DriveForDistanceCmd.__init__(..., distance={distance})")

    def initialize(self):
        self.initialDistance = self.drive.getAverageDistance()
        self.drive.resetEncoders()

    def execute(self):
        self.drive.arcadeDriveSS(0.5, 0.0)  # Example speed, adjust as necessary
        print(f"DriveForDistanceCmd.execute() - Driving...")

    def end(self, interrupted):
        self.drive.arcadeDriveSS(0.0, 0.0)
        print(f"DriveForDistanceCmd.end() - Stopping...")

    def isFinished(self):
        # Use the average distance from both sides to check if the target distance is reached
        currentAverageDistance = self.drive.getAverageDistance()
        fin = abs(currentAverageDistance) >= abs(self.distance)
        print(f"DriveForDistanceCmd.end().isFinished() - cDD: {currentAverageDistance}, dist: {self.distance}, fin={fin}")
        return fin
