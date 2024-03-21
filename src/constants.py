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
    steeringScaleFactor = 0.8

class armConsts:
    # PID Values
    pGain = 6.0 # P was 6 and 7   ; Retuning starting at 0.2 - try bringing this down  # formerly called rotationSpeedScaler
    iGain = 0.7  # I  start with 1/100th of P
    dGain = 0.15  # D  start with 1/10th of P
    gravityGain = 0.9   # Was 0.5 and 0.75 before ; Retuning starting at 0.2 --> 0.7 seems to be the optimal value in Brake Mode ; Coast mode requires 0.7 as well
    gravityDeadband = 0.10
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
    armMaxStallCurrent = 40   # started with 30, but the arm couldn't raise itself
    armMaxFreeCurrent = 40    # started with 30, but the arm couldn't raise itself

class shootingConsts:
    speakerPosition = 0.3
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
    driveSpeed = 0.4  # TODO: At comp we need to figure out what speed works best
    driveBackTime = 5  # TODO: At comp we need to figure out what time works best
    driveTimeSubToMiddleNote = 3  # TODO: If we have time, at comp we need to figure out what time works best
    simpleDriveTimeOutOfZone = 3  # TODO: At comp we need to figure out what time works best