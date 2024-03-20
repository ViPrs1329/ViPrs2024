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
    maxStallCurrent = 30
    maxFreeCurrent = 30

class armConsts:
    # PID Values
    pGain = 7 # was 6   ; this is P   - try bringing this down  # formerly rotationSpeedScaler
    iGain = 0.0  # I  start with 0.001
    dGain = 0.0  # D  start with 0.01
    gravityGain = 0.5   # Was 0.5 and 0.75 before ; Try startign with 0.2 
    gravityDeadband = 0.1
    downPosition = 0.0
    upPosition = Stacy.pi/2.0
    radiansPerRev = 2 * Stacy.pi
    countsPerRev = 2048
    motorToArmGearRatio = 82.5 # to 1
    intakeAngle = 0.0 # radians
    speakerAngle = 0.4 # radians
    ampAngle = 1.6 # was 1.5 radians
    dampingConstant = 1
    intakeSpeed = 0.8
    intakeMaxStallCurrent = 25
    intakeMaxFreeCurrent = 25
    armMaxStallCurrent = 30
    armMaxFreeCurrent = 30

class shootingConsts:
    speakerPosition = 0.35
    ampPositon = 1.5 # was 1.88
    startingPosition = 1.5
    safePosition = 0
    shootingSpeedTop = 1
    shootingSpeedBottom = 1
    ampSpeed = 0.15
    backupTime = 0.025
    backupSpeed = -0.5
    shooterMaxStallCurrent = 30
    shooterMaxFreeCurrent = 30

class autoConsts:
    # driveTime = 3  # TODO: At comp we need to figure out what time works best
    driveSpeed = 0.2  # TODO: At comp we need to figure out what speed works best
    driveBackTime = 5  # TODO: At comp we need to figure out what time works best
    driveTimeSubToMiddleNote = 3  # TODO: If we have time, at comp we need to figure out what time works best
    simpleDriveTimeOutOfZone = 4  # TODO: At comp we need to figure out what time works best