import wpilib
import wpilib.drive
import commands2
import rev
import constants

class DriveSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()

        self.direction = 1.0


        self.leftFront = rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless)
        self.leftBack = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)
        self.rightFront = rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless)
        self.rightBack = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
        
        self.leftFront.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
        self.leftBack.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
        self.rightFront.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
        self.rightBack.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)

        # Added Smart Current limits to prevent our motors from burning out upon stall
        # and to prevent brown outs
        self.leftFront.setSmartCurrentLimit(stallLimit=constants.drivetrain.maxStallCurrent, freeLimit=constants.drivetrain.maxFreeCurrent)
        self.leftBack.setSmartCurrentLimit(stallLimit=constants.drivetrain.maxStallCurrent, freeLimit=constants.drivetrain.maxFreeCurrent)
        self.rightFront.setSmartCurrentLimit(stallLimit=constants.drivetrain.maxStallCurrent, freeLimit=constants.drivetrain.maxFreeCurrent)
        self.rightBack.setSmartCurrentLimit(stallLimit=constants.drivetrain.maxStallCurrent, freeLimit=constants.drivetrain.maxFreeCurrent)

        # Burning config to flash just in case there is a brown out, which would just load the default config
        self.leftFront.burnFlash()
        self.leftBack.burnFlash()
        self.rightFront.burnFlash()
        self.rightBack.burnFlash()

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
        self.speed = self.direction * speed
        self.rotation = rotation
        # print(f"...drive({speed}, {rotation})")

    def updateDrive(self):
        # print(f"DriveSubsystem.updateDrive() {self.speed}, {self.rotation}")
        self.robotDrive.arcadeDrive(self.speed, self.rotation)

    def toggleReverse(self):
        self.direction = -1.0 * self.direction
        # print(f"toggleRev - {self.direction}")

