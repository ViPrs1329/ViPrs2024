import numpy as Stacy
from enum import Enum, auto
from team4646.PID import PID


# We should add CAN IDs here
class CANIDs:
    leftDriveSparkFront = 4
    leftDriveSparkBack = 3
    rightDriveSparkFront = 2
    rightDriveSparkBack = 1
    leftArmSpark = 6
    rightArmSpark = 5
    topShootingSpark = 7
    bottomShootingSpark = 8
    intakeSpark = 9

class inputConsts:
    inputScale = 0.8
    inputDeadZone = 0.1
    rampRate = 0.05

class convert:
    def in2m(inches):
        return 0.0254 * inches
    
    def rev2rad(rev):
        return rev * 2 * Stacy.pi

    def rad2rev(radians):
        return radians / (2 * Stacy.pi)
    
    def count2rev(count):
        return count / armConsts.countsPerRev
    
class driveConsts:
    wheelDiameter = convert.in2m(6)
    driveMaxOutput = 0.8 
    slowDriveScale = 0.5

class armConsts:
    rotationSpeedScaler = 0.1
    # armControlP = .06
    # armControlI = 0
    # armControlD = 0
    armPIDValues = PID(
        0.01, # P
        0.0, # I
        0.0 # D
    )
    PIDTolerance = 0.1
    downPosition = 0.0
    upPosition = Stacy.pi/2.0
    radiansPerRev = 2 * Stacy.pi
    gravityGain = 0.5
    countsPerRev = 2048
    motorToArmGearRatio = 82.5 # to 1
    rightEncoder = 5
    leftEncoder = 6
    rightRelativeEncoderA = 3
    rightRelativeEncoderB = 4
    leftRelativeEncoderA = 7
    leftRelativeEncoderB = 8
    maxVelocity = 1.0
    maxAcc = 0.5
    slotID = 0
    intakeAngle = 0.0 # radians
    speakerAngle = 0.4 # radians
    ampAngle = 1.5 # radians
    startingAngle = 1.0 # radians
    maxEncoderValue = 1.55
    minEncoderValue = 0.5
    rightEncoderOffset = 0.42430531060763277
    leftEncoderOffset = 0.5910043147751078

class intakeConsts:
    captureSpeed = 0.75
    releaseSpeed = -0.75
    offSpeed = 0
    deliverToShooterSpeed = 0.75
    deliverToShooterTime = 1.0
    retractSpeed = -0.1
    retractTime = 0.1
    expelSpeed = 0.7
    expelTime = 0.5

class shooterConsts:
    topShootHighSpeed = 0.75
    bottomShootHighSpeed = 0.75
    topShootLowSpeed = 0.25
    bottomShootLowSpeed = 0.25

class sensorConsts:
    noteSensorDIO = 2
    armBottomLimit = 1
    armTopLimit = 9

class States(Enum):
        # Define general states that might apply to various subsystems or the robot in general
        IDLE = auto()
        COLLECTING = auto()
        SHOOTING = auto()
        MOVING = auto()
        # etc.
        
        # You can also define specific states for each subsystem if needed
        INTAKE_COLLECTING = auto()
        INTAKE_IDLE = auto()
        INTAKE_RETRACTING = auto()
        INTAKE_UNJAM = auto()
        INTAKE_EJECTING = auto()
        SHOOTER_PREPPING = auto()
        SHOOTER_FIRING = auto()
        SHOOTER_IDLE = auto()
        ARM_COLLECTING = auto()
        ARM_SCORING_LOW = auto()
        ARM_SCORING_HIGH = auto()
        ARM_IDLE = auto()
        ARM_START_STATE = auto()

