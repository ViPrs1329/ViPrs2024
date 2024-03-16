from commands2 import Command
from subSystems.REVDriveSubsystem import DriveSubsystem
from constants import driveConsts

class RotateToAngle(Command):
    def __init__(self, drive: DriveSubsystem, angle: float):
        super().__init__()
        self.drive = drive
        self.targetAngle = angle
        self.kP = driveConsts.driveRotationPIDValues.P
        # self.kI = driveConsts.driveRotationPIDValues.I
        # self.kD = driveConsts.driveRotationPIDValues.D
        self.addRequirements(drive)
        print(f"RotateToAngle.__init__(..., angle={angle})")

    def initialize(self):
        self.drive.resetGyro()  

    def execute(self):
       # Calculate the current error between target angle and current angle
        currentAngle = self.drive.gyroAccl.getYaw()
        error = self.targetAngle - currentAngle
        
        # Calculate rotation speed based on proportional control
        rotationSpeed = self.kP * error
        
        # Use the rotation speed to rotate the robot
        self.drive.arcadeDriveSS(0.0, rotationSpeed)

        print(f"RotateToAngle.execute() - cA: {round(currentAngle, 2)}, er: {round(error, 2)}, rS: {round(rotationSpeed, 2)}")

    def end(self, interrupted):
        self.drive.arcadeDriveSS(0.0, 0.0)

    def isFinished(self):
        # Consider command finished when the robot is within a small error margin of the target angle
        currentAngle = self.drive.gyroAccl.getYaw()
        error = self.targetAngle - currentAngle
        fin = abs(error) < driveConsts.driveRotationAcceptableError
        print(f"RotateToAngle.isFinished() - cA: {round(currentAngle, 2)}, er: {round(error, 2)}, fin???: {fin}")
        return fin
