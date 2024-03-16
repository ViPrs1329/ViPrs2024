import wpilib
import wpilib.drive
import commands2
import rev
import navx
import math
import constants

from team254.SparkMaxFactory import SparkMaxFactory
from team254.LazySparkMax import LazySparkMax

 
class DriveSubsystem(commands2.Subsystem):
    class Cache:
        def __init__(self):
            self.forwardSetpoint = 0.0
            self.rotationSetpoint = 0.0
            self.gyroAngle = 0.0
            self.leftFrontCurrentAmp = 0.0
            self.leftBackCurrentAmp = 0.0
            self.rightFrontCurrentAmp = 0.0
            self.rightBackCurrentAmp = 0.0
            self.leftFrontEncoderValue = 0.0
            self.leftBackEncoderValue = 0.0
            self.rightFrontEncoderValue = 0.0
            self.rightBackEncoderValue = 0.0

    def __init__(self):
        super().__init__()

        self.cache = self.Cache()

        self.isSlowMode = False
        self.isReverseMode = False

        self.gyroAccl = navx.AHRS(wpilib.SPI.Port.kMXP)
        
        
        # self.leftFront = rev.CANSparkMax(constants.CANIDs.leftDriveSparkFront, rev.CANSparkMax.MotorType.kBrushless)
        # self.leftBack = rev.CANSparkMax(constants.CANIDs.leftDriveSparkBack, rev.CANSparkMax.MotorType.kBrushless)
        # self.rightFront = rev.CANSparkMax(constants.CANIDs.rightDriveSparkFront, rev.CANSparkMax.MotorType.kBrushless)
        # self.rightBack = rev.CANSparkMax(constants.CANIDs.rightDriveSparkBack, rev.CANSparkMax.MotorType.kBrushless)

        self.leftFront = SparkMaxFactory.createDefaultSparkMax(constants.CANIDs.leftDriveSparkFront)
        self.leftBack = SparkMaxFactory.createDefaultSparkMax(constants.CANIDs.leftDriveSparkBack)
        self.rightFront = SparkMaxFactory.createDefaultSparkMax(constants.CANIDs.rightDriveSparkFront)
        self.rightBack = SparkMaxFactory.createDefaultSparkMax(constants.CANIDs.rightDriveSparkBack)
        
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

        # Set up encoders
        self.leftFrontEncoder = self.leftFront.getEncoder()
        self.leftBackEncoder = self.leftBack.getEncoder()
        self.rightFrontEncoder = self.rightFront.getEncoder()
        self.rightBackEncoder = self.rightBack.getEncoder()

        # Calculate conversion factors for encoders
        wheelCircumference = math.pi * constants.driveConsts.wheelDiameter
        gearboxRatio = constants.driveConsts.gearboxRatio
        positionConversionFactor = wheelCircumference / gearboxRatio

        # Set the conversion factor for both left and right encoders
        self.leftFrontEncoder.setPositionConversionFactor(positionConversionFactor)
        self.leftBackEncoder.setPositionConversionFactor(positionConversionFactor)
        self.rightFrontEncoder.setPositionConversionFactor(positionConversionFactor)
        self.rightBackEncoder.setPositionConversionFactor(positionConversionFactor)

        self.resetEncoders()


    def toggleSlowMode(self):
        self.isSlowMode = not self.isSlowMode

    def toggleReverseMode(self):
        self.isReverseMode = not self.isReverseMode

    def getDriveScale(self):
        """Return the scale factor based on the slow mode state."""
        if self.isSlowMode:
            return constants.driveConsts.slowDriveScale
        else:
            return 1.0  # No scaling in normal mode


    def setMaxOutput(self, maxOutput: float):
        """
        Sets the max output of the drive. Useful for scaling the drive to drive more slowly.

        :param maxOutput: the maximum output to which the drive will be constrained
        """
        self.robotDrive.setMaxOutput(maxOutput)

    def arcadeDriveSS(self, forward, rotation):
        # self.robotDrive.arcadeDrive(forward, rotation)

        if self.isReverseMode:
            forward = -forward

        scale = self.getDriveScale()
        self.cache.forwardSetpoint = forward * scale
        self.cache.rotationSetpoint = rotation * scale

    def updateHardware(self):
        self.robotDrive.arcadeDrive(self.cache.forwardSetpoint, self.cache.rotationSetpoint)

    def getLeftEncoderDistance(self):
        """
        Calculate the distance traveled based on the encoder counts from the left side of the drive.
        Takes into account the gearbox ratio to convert motor rotations to wheel rotations, and then
        converts those rotations to linear distance traveled.
        """
        # wheelDiameter = constants.driveConsts.wheelDiameter
        # wheelCircumference = math.pi * wheelDiameter
        # gearboxRatio = constants.driveConsts.gearboxRatio

        # Get the encoder positions in terms of motor rotations
        # leftFrontMotorRotations = self.leftFrontEncoder.getPosition()
        # leftBackMotorRotations = self.leftBackEncoder.getPosition()
        # leftFrontMotorRotations = self.cache.leftFrontEncoderValue
        # leftBackMotorRotations = self.cache.leftBackEncoderValue

        # Convert motor rotations to wheel rotations by dividing by the gearbox ratio
        # leftFrontWheelRotations = leftFrontMotorRotations / gearboxRatio
        # leftBackWheelRotations = leftBackMotorRotations / gearboxRatio

        # Convert wheel rotations to linear distance
        # leftFrontDistance = leftFrontWheelRotations * wheelCircumference
        # leftBackDistance = leftBackWheelRotations * wheelCircumference

        # Average the distances from the front and back encoders for a more representative value
        leftFrontDistance = self.cache.leftFrontEncoderValue
        leftBackDistance = self.cache.leftBackEncoderValue
        averageLeftDistance = (leftFrontDistance + leftBackDistance) / 2

        return averageLeftDistance

    def getRightEncoderDistance(self):
        """
        Calculate the distance traveled based on the encoder counts from the right side of the drive.
        Takes into account the gearbox ratio to convert motor rotations to wheel rotations, and then
        converts those rotations to linear distance traveled.
        """
        # wheelDiameter = constants.driveConsts.wheelDiameter
        # wheelCircumference = math.pi * wheelDiameter
        # gearboxRatio = constants.driveConsts.gearboxRatio

        # Get the encoder positions in terms of motor rotations
        # rightFrontMotorRotations = self.rightFrontEncoder.getPosition()
        # rightBackMotorRotations = self.rightBackEncoder.getPosition()  
        # rightFrontMotorRotations = self.cache.rightFrontEncoderValue
        # rightBackMotorRotations = self.cache.rightBackEncoderValue

        # Convert motor rotations to wheel rotations by dividing by the gearbox ratio
        # rightFrontWheelRotations = rightFrontMotorRotations / gearboxRatio
        # rightBackWheelRotations = rightBackMotorRotations / gearboxRatio

        # Convert wheel rotations to linear distance
        # rightFrontDistance = rightFrontWheelRotations * wheelCircumference
        # rightBackDistance = rightBackWheelRotations * wheelCircumference

        # Average the distances from the front and back encoders for a more representative value
        rightFrontDistance = self.cache.rightFrontEncoderValue
        rightBackDistance = self.cache.rightBackEncoderValue
        averageRightDistance = (rightFrontDistance + rightBackDistance) / 2

        return abs(averageRightDistance)

    def getAverageDistance(self):
        """
        Calculate the average distance traveled by both the left and right sides of the drive.
        This gives a general idea of how far the robot has moved forward.
        """
        averageLeftDistance = self.getLeftEncoderDistance()
        averageRightDistance = self.getRightEncoderDistance()

        # print(f"DriveSubsystem.getAverageDistance() - aLD: {averageLeftDistance}, aRD: {averageRightDistance}")

        return (averageLeftDistance + averageRightDistance) / 2

    def getDrift(self):
        """
        Calculate the difference in distance between the left and right sides of the drive.
        A non-zero value indicates the robot is drifting to one side, which could be used
        to correct its path if necessary.
        """
        averageLeftDistance = self.getLeftEncoderDistance()
        averageRightDistance = self.getRightEncoderDistance()

        return averageRightDistance - averageLeftDistance


    def resetGyro(self):
        """
        Resets the gyro's yaw angle to zero. This makes the robot's current direction as the "forward" direction.
        """
        self.gyroAccl.zeroYaw()

    def resetEncoders(self):
        self.leftBackEncoder.setPosition(0)
        self.leftFrontEncoder.setPosition(0)
        self.rightBackEncoder.setPosition(0)
        self.rightFrontEncoder.setPosition(0)

    def getDriveRotation(self):
        """
        Returns the angle of the robot since the last reset
        """
        return self.cache.gyroAngle

    def cacheSensors(self):
        self.cache.gyroAngle = self.gyroAccl.getAngle()
        self.cache.leftFrontCurrentAmp = self.leftFront.getOutputCurrent()
        self.cache.leftBackCurrentAmp = self.leftBack.getOutputCurrent()
        self.cache.rightFrontCurrentAmp = self.rightFront.getOutputCurrent()
        self.cache.rightBackCurrentAmp = self.rightBack.getOutputCurrent()
        self.cache.leftFrontEncoderValue = self.leftFrontEncoder.getPosition()
        self.cache.leftBackEncoderValue = self.leftBackEncoder.getPosition()
        self.cache.rightFrontEncoderValue = self.rightFrontEncoder.getPosition()
        self.cache.rightBackEncoderValue = self.rightBackEncoder.getPosition()
        # print(f"DriveSubsystem.cacheSensors() - lFEV: {round(self.cache.leftFrontEncoderValue, 2)}, lBEV: {round(self.cache.leftBackEncoderValue, 2)}, rFEV: {round(self.cache.rightFrontEncoderValue, 2)}, rBEV: {round(self.cache.rightBackEncoderValue, 2)}")
        