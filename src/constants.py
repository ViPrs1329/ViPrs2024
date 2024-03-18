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

class armConsts:
    rotationSpeedScaler = 6 # 0.5
    downPosition = 0.0
    upPosition = Stacy.pi/2.0
    radiansPerRev = 2 * Stacy.pi
    gravityGain = 0.75
    countsPerRev = 2048
    motorToArmGearRatio = 82.5 # to 1
    intakeAngle = 0.0 # radians
    speakerAngle = 0.4 # radians
    ampAngle = 1.5 # was 1.5 radians
    dampingConstant = 1
    intakeSpeed = 0.8
    gravityDeadband = 0.1

class shootingConsts:
    speakerPosition = 0.245
    ampPositon = 1.6 # was 1.88
    startingPosition = 1.5
    safePosition = 0
    shootingSpeedTop = 1
    shootingSpeedBottom = 1
    backupTime = 0.1
    backupSpeed = -0.5

class autoConsts:
    driveTime = 3
    driveSpeed = 0.2
    driveBackTime = 5