import wpilib
import wpilib.drive
import commands2
import rev
import constants

class DriveSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.leftFront = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)
        self.leftBack = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)
        self.rightFront = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless)
        self.rightBack = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
        
        self.leftFront.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
        self.leftBack.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
        self.rightFront.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
        self.rightBack.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)

        self.leftDrive = wpilib.MotorControllerGroup(self.leftFront, self.leftBack)
        self.rightDrive = wpilib.MotorControllerGroup(self.rightFront, self.rightBack)
        
        # self.spark1 = rev.CANSparkMax(5, rev._rev.CANSparkLowLevel.MotorType.kBrushless)
        # self.drive1 = wpilib.drive.RobotDriveBase
        self.robotDrive = wpilib.drive.DifferentialDrive(
            self.leftDrive, self.rightDrive
        )
        self.rightDrive.setInverted(True)

        self.leftFrontEncoder = self.leftFront.getEncoder()

        self.speed = 0
        self.rotation = 0

    def drive(self, speed, rotation):
        self.speed = speed
        self.rotation = rotation

    def updateDrive(self):
        self.robotDrive.arcadeDrive(self.speed, self.rotation)
        
    