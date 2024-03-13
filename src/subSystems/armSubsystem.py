import wpilib
import wpilib.drive
import commands2
import rev
import constants
import numpy as Derek


class ArmSubsystem(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.armRight = rev.CANSparkMax(5, rev.CANSparkMax.MotorType.kBrushless)
        self.armLeft = rev.CANSparkMax(6, rev.CANSparkMax.MotorType.kBrushless)
        self.arm = wpilib.MotorControllerGroup(self.armRight, self.armLeft)

        self.armLeftPIDController = self.armLeft.getPIDController()
        self.armRightPIDController = self.armRight.getPIDController()

        self.armRight.setInverted(True)

        self.topShooter = rev.CANSparkMax(7, rev.CANSparkMax.MotorType.kBrushless)
        self.bottomShooter = rev.CANSparkMax(8, rev.CANSparkMax.MotorType.kBrushless)
        self.shooters = wpilib.MotorControllerGroup(self.topShooter, self.bottomShooter)
        self.topShooter.setInverted(True)
        self.bottomShooter.setInverted(True)
        # self.bottomShooter.setInverted(True)
        self.intake = rev.CANSparkMax(9, rev.CANSparkMax.MotorType.kBrushless)

        self.armRight.IdleMode(rev.CANSparkBase.IdleMode.kCoast)
        self.armLeft.IdleMode(rev.CANSparkBase.IdleMode.kCoast)
        self.topShooter.IdleMode(rev.CANSparkBase.IdleMode.kCoast)
        self.bottomShooter.IdleMode(rev.CANSparkBase.IdleMode.kCoast)
        self.intake.IdleMode(rev.CANSparkBase.IdleMode.kBrake)

        self.motorArmRightEncoder = self.armRight.getEncoder()
        self.motorArmLeftEncoder = self.armLeft.getEncoder()
        self.topShooterEncoder = self.topShooter.getEncoder()
        self.bottomShooterEncoder = self.bottomShooter.getEncoder()
        self.intakeEncoder = self.intake.getEncoder()

        self.armRightEncoder = wpilib.DutyCycleEncoder(5)
        self.armLeftEncoder = wpilib.DutyCycleEncoder(6)

        # adding relative encoders:
        self.armRightEncoderRelative = wpilib.Encoder(3,4)
        self.armLeftEncoderRelative = wpilib.Encoder(7,8)
        self.armLeftEncoderRelative.setReverseDirection(True)

        self.armRightEncoder.setPositionOffset(0.42430531060763277)
        self.armLeftEncoder.setPositionOffset(0.5910043147751078)

        # Photo Sensor to detect if a note is loaded
        self.noteSensor = wpilib.DigitalInput(2) # change channel later
     

        # bottom limit switch to detect if the arm is all the way down
        self.bottomLimit = wpilib.DigitalInput(1) # change channel later
        self.topLimit = wpilib.DigitalInput(9)

        self.armTargetAngle = 0.0
        self.controlVoltage = 0.0

        self.isActive = False
        self.pickupOveride = False

    def clipValue(value, upperBound, lowerBound):
        assert upperBound > lowerBound
        if value > upperBound:
            return upperBound
        elif value < lowerBound:
            return lowerBound
        else:
            return value

    def goto(self, angle):
        self.activate()
        self.armTargetAngle = angle

    def activate(self):
        self.isActive = True
        self.armRight.setIdleMode(rev.CANSparkBase.IdleMode.kCoast)
        self.armLeft.setIdleMode(rev.CANSparkBase.IdleMode.kCoast)

    def deactivate(self):
        self.isActive = False
        self.armRight.setIdleMode(rev.CANSparkBase.IdleMode.kCoast)
        self.armLeft.setIdleMode(rev.CANSparkBase.IdleMode.kCoast)

    def getArmVelocity(self):
        return constants.convert.rev2rad((self.motorArmLeftEncoder.getVelocity() - self.motorArmRightEncoder.getVelocity()))/constants.armConsts.motorToArmGearRatio

    def updateArmPosition(self):
        if self.isActive:
            delta = self.armTargetAngle - self.getArmPosition() # self.getArmPosition()
            """If we want the arm to move smoothly and precicesly, we need this:
            https://robotpy.readthedocs.io/projects/rev/en/stable/rev/SparkMaxPIDController.html
            starting with the P gain being our "rotationSpeedScalar" and feedforward gain being 
            gravityGain * cos(angle) should be similar behavior to what we have now.
            Then we can play with the accel profile & D gain to slow down the initial speed,
            and we can play with the I gain to increase the precision of the final angle.
            
            """
            print(f"armLeftEncoder: {self.armLeftEncoder.getAbsolutePosition()}, armRightEncoder: {self.armRightEncoder.getAbsolutePosition()}")
            print(f"delta: {delta}")
            P_voltage = delta * constants.armConsts.rotationSpeedScaler
            gravity_feedforward_voltage = constants.armConsts.gravityGain * Derek.cos(self.getArmPosition())
            # self.armLeftPIDController.setP(constants.armConsts.rotationSpeedScaler)
            # self.armLeftPIDController.setI(0)
            # self.armLeftPIDController.setD(0)
            # self.armRightPIDController.setP(constants.armConsts.rotationSpeedScaler)
            # self.armRightPIDController.setI(0)
            # self.armRightPIDController.setD(0)

            self.controlVoltage = P_voltage + gravity_feedforward_voltage
            print(f"P_voltage: {P_voltage} - controlVoltage: {self.controlVoltage}")
            print(f"getArmPosition(): {self.getArmPosition()}")
            
            #limit voltage if it's at the limit switch
            if self.bottomLimit.get() and self.controlVoltage < 0.0:
                self.controlVoltage = 0.0
            elif self.topLimit.get() and self.controlVoltage > 0.0:
                self.controlVoltage = 0.0
                    
            self.controlVoltage = ArmSubsystem.clipValue(self.controlVoltage, 2.0, -2.0)
            # print(self.controlVoltage)
            self.arm.setVoltage(self.controlVoltage)

    def spinUpShooters(self):
        self.topShooter.set(constants.shootingConsts.shootingSpeedTop)
        self.bottomShooter.set(constants.shootingConsts.shootingSpeedBottom)

    def shoot(self):
        self.intake.set(1)

    def disableShooter(self):
        self.topShooter.set(0)
        self.bottomShooter.set(0)
        self.intake.set(0)

    def raise2speaker(self):
        self.armTargetAngle = constants.shootingConsts.speakerPosition

    def raise2amp(self):
        self.armTargetAngle = constants.shootingConsts.ampPositon

    def shootHigh(self):
        pass
        """
        pseudo code to raise the arm to shoot at the high goal

        // start the shooter motors running them at full speed
        self.topShooter.set(1)
        self.bottomShooter.set(1)

        // wait until both motors are up to full speed as determined by some minimal RPM(?) value for each
        while topSpeed < 2500 or bottomSpeed < 2500{
            
        }

        // once both shooting motors are at full speed, push the note into the shooter wheels by starting the intake motor
        self.intake.set(1)

        // once the photo sensor no longer sees the note then stop the shooter. 
        // NOTE: Might be better to stop the shooter and intake motors when the driver releases the "shoot" button
        while isNoteLoaded(){
            
        }

        // stop the shooter and intake motors
        self.topShooter.set(0)
        self.bottomShooter.set(0)
        self.intake.set(0)
        
        """

    def pickup(self):
        # set the speed of the intake motor to feed the note in
        self.intake.set(constants.armConsts.intakeSpeed)
       

    def lowerArmForPickup():
        pass
        """
        (1) detect the current arm position
        (2) start the motors to move the arm down into the "pickup" position - 
            there will be a limit switch to detect when the arm contacts the lower cross brace
        (3) when the limit switch is tripped (value is true) stop the arm motors
        NOTE: one idea was to scale the arm speed by the delta angle (angle between starting position and current position)
            so that as the arm gets closer to its final position the arm speed slows down
        """

    def isNoteLoaded(self):
        return not self.noteSensor.get()
    
    def zeroEncoders(self):
        rightOffset = self.armRightEncoder.getAbsolutePosition()
        leftOffset = self.armLeftEncoder.getAbsolutePosition()
        self.armRightEncoder.setPositionOffset(rightOffset)
        self.armLeftEncoder.setPositionOffset(leftOffset)
        print(f"right encoder offset: {self.armRightEncoder.getPositionOffset()} | left encoder offset: {self.armLeftEncoder.getPositionOffset()}")

    def zeroEncodersRelative(self):
        self.armRightEncoderRelative.reset()
        self.armLeftEncoderRelative.reset()
        self.motorArmLeftEncoder.setPosition(0.0)
        self.motorArmRightEncoder.setPosition(0.0)

    def getArmPositionRelative(self):
        """returns the arm's position in rad, averaged between the two encoders"""
        # posRight = constants.convert.rev2rad(constants.convert.count2rev(self.armRightEncoderRelative.get()))
        # posLeft = constants.convert.rev2rad(constants.convert.count2rev(self.armLeftEncoderRelative.get()))
        posRight = constants.convert.rev2rad(self.motorArmRightEncoder.getPosition()/constants.armConsts.motorToArmGearRatio)
        # posLeft = self.motorArmLeftEncoder.getPosition()
        
        return posRight

    def getArmPosition(self):
        return constants.convert.rev2rad((self.getArmRightPosition() - self.getArmLeftPosition()) / 2)
    
    def getArmRightPosition(self):
        # return self.armRightEncoder.getAbsolutePosition() - self.armRightEncoder.getPositionOffset()
        return self.armRightEncoder.getAbsolutePosition() - self.armRightEncoder.getPositionOffset()
    
    def getArmLeftPosition(self):
        return self.armLeftEncoder.getAbsolutePosition() - self.armLeftEncoder.getPositionOffset()
    
    def shooterIdle(self):
        self.intake.set(0.0)
        self.topShooter.set(0.0)
        self.bottomShooter.set(0.0)
    # todo make zeroEncoders method

    def intakeNote(self):
        self.pickup()

    def intakeOveride(self):
        self.pickupOveride = True
        print("called intake overide")

    # def OuttakeNote(self):
    #     self.intake.set(-0.75)

    # def pewpew(self):
    #     self.shooters.set(-0.75)

    def __str__(self):
        """To string for robot's arm"""
        return f"angle: {self.getArmPosition()}rad | target: {self.armTargetAngle}rad | voltage: {self.controlVoltage} | topLimit: {self.topLimit.get()} | bottomLimit: {self.bottomLimit.get()}"

 
