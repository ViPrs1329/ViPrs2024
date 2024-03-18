import commands2
from subSystems.armSubsystem import ArmSubsystem
from subSystems.REVDriveSubsystem import DriveSubsystem
import constants
import wpilib
class autoDriveBackwards(commands2.Command):
    def __init__(self, driveSubsystem: DriveSubsystem):
        super().__init__()
        self.driveSubsystem = driveSubsystem
        self.initialTime = 0
        self.timer = wpilib.Timer()
        self.timePassed = 0

    def initialize(self):
        self.timer.reset()
        self.timer.start()
        self.initialTime = wpilib.Timer.getFPGATimestamp()
        self.timePassed = self.timer.get()

    def execute(self):
        print(f"aDB.execute()")
        self.driveSubsystem.drive(-1.0 * constants.autoConsts.driveSpeed, 0)
        # self.driveSubsystem.robotDrive.arcadeDrive(constants.autoConsts.driveSpeed, 0, False)
        # if self.timer.get() - self.timePassed > 0.002:
            # self.driveSubsystem.robotDrive.arcadeDrive(-1.0 * constants.autoConsts.driveSpeed, 0, False)
            # self.driveSubsystem.drive(-1.0 * constants.autoConsts.driveSpeed, 0)
            # self.timePassed = self.timer.get()

    # def isFinished(self):
    #     hasElapsed = self.timer.hasElapsed(constants.autoConsts.driveBackTime)
    #     print(hasElapsed)
    #     return hasElapsed
    
    # def end(self, interrupted):
    #     self.timer.stop()
        # self.driveSubsystem.robotDrive.arcadeDrive(0, 0, False)
        # self.driveSubsystem.drive(0, 0, False)
