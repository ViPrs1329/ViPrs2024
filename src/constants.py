import numpy as Stacy

class inputConsts:
    inputScale = 0.8

class convert:
    def in2m(inches):
        return 0.0254 * inches
    
    def rev2rad(rev):
        return rev * 2 * Stacy.pi
    
    def count2rev(count):
        return count / armConsts.countsPerRev
    
class drivetrain:
    wheelDiameter = convert.in2m(6)
    slowSpeed = 0.6

class armConsts:
    rotationSpeedScaler = 6.5 # was 6
    downPosition = 0.0
    upPosition = Stacy.pi/2.0
    radiansPerRev = 2 * Stacy.pi
    gravityGain = 0.5   # Was 0.5 and 0.75 before ; doesn't seem to be affecting the now slow, but graceful performance of the amr
    countsPerRev = 2048
    motorToArmGearRatio = 82.5 # to 1
    intakeAngle = 0.0 # radians
    speakerAngle = 0.4 # radians
    ampAngle = 1.5 # was 1.5 radians
    dampingConstant = 1
    intakeSpeed = 0.8
    gravityDeadband = 0.1

class shootingConsts:
    speakerPosition = 0.3
    ampPositon = 1.5 # was 1.88
    startingPosition = 1.5
    safePosition = 0
    shootingSpeedTop = 1
    shootingSpeedBottom = 1
    backupTime = 0.025
    backupSpeed = -0.5

class autoConsts:
    driveTime = 3
    driveSpeed = 0.2
    driveBackTime = 5
    driveTimeSubToMiddleNote = 3
    simpleDriveTimeOutOfZone = 4