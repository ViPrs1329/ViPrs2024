import commands2
import commands2.button
import commands2.cmd
import numpy as DrArnett
from wpimath import filter

from subSystems.REVDriveSubsystem import DriveSubsystem
from subSystems.armSubsystem import ArmSubsystem
from commands.shootNote import ShootNote
from commands.stopShooter import StopShooter
from commands.retractNote import Backup
from commands.pickupNote import PickupNote
from commands.detectNote import DetectNote
from commands.gotoTaxiPosition import GotoTaxiPosition
from commands.autoDriveForward import autoDriveForward
from commands.autoDriveBackwards import autoDriveBackwards
from commands.stopDriveCommand import StopDriveCommand
from commands.toggleReverse import ToggleReverse
from commands.toggleSlow import ToggleSlow
from commands.shootNoteSlow import ShootNoteSlow
from commands.extractNote import Extract
from commands.unscheduleShooting import Unschedule

from wpilib import XboxController

import constants

def shapeInputs(input, scale_factor):
    def y1(x):
        return (DrArnett.sin((x * DrArnett.pi) - (DrArnett.pi / 2)) / 2) + 0.5
    def y2(x):
        return (DrArnett.sin((x * DrArnett.pi) - (DrArnett.pi / 2)) / -2) - 0.5
    return (y1(input) * int(input >= 0) * (input <= 1) + y2(input) * (input >= -1) * (input <= 0)) * scale_factor * constants.inputConsts.inputScale

class RobotContainer:
    
    def __init__(self):
        self.driverControler = commands2.button.CommandXboxController(0)
        self.wpiXboxController = XboxController(0)
        self.robotDrive = DriveSubsystem()
        self.arm = ArmSubsystem()
        self.shootNoteObject = ShootNote(self.arm)
        self.stopShooterObject = StopShooter(self.arm)

        self.shootNoteObjectSlow = ShootNoteSlow(self.arm)
        self.stopShooterObjectSlow = StopShooter(self.arm)
        self.backupObject = Backup(self.arm)
        self.pickupObject = PickupNote(self.arm, self.wpiXboxController)
        self.detectNoteObject = DetectNote(self.arm, self.wpiXboxController)
        self.inputFilter = filter.SlewRateLimiter(2)
        self.gotoTaxiPositionObject = GotoTaxiPosition(self.arm)
        self.taxiFromAmp = GotoTaxiPosition(self.arm)
        self.direction = 1
        self.isSlow = False
        self.configureButtonBindings()
        
        self.scale_factor = 1
        # self.robotDrive.setDefaultCommand(
        #     commands2.cmd.run(
        #         lambda: self.robotDrive.robotDrive.arcadeDrive(
        #             self.direction * self.inputFilter.calculate(
        #                 shapeInputs(
        #                     -self.driverControler.getLeftY(), self.scale_factor
        #                 )
        #             ),
        #             shapeInputs(
        #                 -self.driverControler.getRightX(), self.scale_factor
        #             )
        #         ),
        #         self.robotDrive
        #     )
        #     # .alongWith(
        #     #     commands2.cmd.run(
        #     #         lambda: self.arm.shooterIdle()
        #     #     )
        #     # )
            
        # )

        # lambda: self.robotDrive.robotDrive.arcadeDrive(
                #     self.direction * self.inputFilter.calculate(
                #         shapeInputs(
                #             -self.driverControler.getLeftY(), self.scale_factor
                #         )
                #     ),
                #     shapeInputs(
                #         -self.driverControler.getRightX(), self.scale_factor
                #     )
                # ),
        
    def setTeleopDefaultCommand(self):
        self.robotDrive.setDefaultCommand(
            commands2.cmd.run(
                lambda: self.robotDrive.drive(
                    self.direction * self.inputFilter.calculate(
                        shapeInputs(
                            -self.driverControler.getLeftY(), self.scale_factor
                        )
                    ),
                    shapeInputs(
                        -self.driverControler.getRightX(), self.scale_factor
                    ) * constants.drivetrain.steeringScaleFactor
                ),
                self.robotDrive
            )
        )
        ''' This is not how you set the default command VVV
        return commands2.cmd.run(
            lambda: self.robotDrive.robotDrive.arcadeDrive(
                self.direction * self.inputFilter.calculate(
                    shapeInputs(
                        -self.driverControler.getLeftY(), self.scale_factor
                    )
                ),
                shapeInputs(
                    -self.driverControler.getRightX(), self.scale_factor
                )
            ),
            self.robotDrive
        )
        '''
    
    # def setAutoDefaultCommand(self):
    #     self.robotDrive.setDefaultCommand(
    #         commands2.cmd.run(
    #             lambda: self.robotDrive.robotDrive.arcadeDrive(0, 0),
    #             self.robotDrive
    #         )
    #     )
    #     self.robotDrive.setDefaultCommand(None)
    
    def configureButtonBindings(self):
        def spinUpShooters():
            self.arm.spinUpShooters()
        
        def spinUpShootersSlow():
            self.arm.spinUpShootersSlow()

    #     self.driverControler.rightBumper().whileTrue(
    #         commands2.cmd.run(
    #             lambda: self.go_slow()
    #         )
    #     )

    #     self.driverControler.rightBumper().whileFalse(
    #         commands2.cmd.run(
    #             lambda: self.go_fast()
    #         )
    #     )

        # Right Bumper
        # Toggle slow mode
        # self.driverControler.rightBumper().onTrue(
        #     ToggleSlow(self) # Assign ToggleSlow command to right bumper
        # )

        # def changeDirection():
        #     print(f"chDir() - {self.direction}")
        #     self.direction = -1.0 * self.direction

        # def backwards():
        #     self.direction = -1
        # def forwards():
        #     self.direction = 1
        self.driverControler.rightBumper().whileTrue(
            commands2.cmd.run(
                lambda: self.go_slow()
            )
        )
        self.driverControler.rightBumper().whileFalse(
            commands2.cmd.run(
                lambda: self.go_fast()
            )
        )

        # Left Bumper
        # Change drive driection
        self.driverControler.leftBumper().onTrue(
            ToggleReverse(self.robotDrive)
        )

        # A button
        # Make arm go to speaker scoring position
        self.driverControler.a().whileTrue(
            commands2.cmd.run(
                lambda: self.arm.goto(constants.shootingConsts.speakerPosition)
            )
        )

        # B button
        # Make arm go to amp scoring positino
        self.driverControler.b().whileTrue(
            commands2.cmd.run(
                lambda: self.arm.goto(constants.shootingConsts.ampPositon)
            )
        )

        # Y button
        # Make arm go to intake position
        self.driverControler.y().whileTrue(
            Extract(self.arm)
        )

        # X button
        # Start a command sequence to:
        # - Call pickup() method in arm subsystem which:
        #   - Turns on the intake
        # - Constantly checks to see if a note has been detected
        # - If it has it backs the note out a bit based on the backup speed and backup time in constants
        # - Go to the taxi position, which is the speaker scoring position so the arm doesn't drag on the ground
        self.driverControler.x().onTrue(
            commands2.cmd.SequentialCommandGroup(
                # Unschedule(self.arm, self.stopShooterObjectSlow),
                self.pickupObject,
                self.detectNoteObject,
                self.backupObject,
                self.gotoTaxiPositionObject
            )
        )

        # Another X button binding... (not sure why)
        # Lowers arm to intake position
        self.driverControler.x().whileTrue(
            commands2.cmd.run(
                lambda: self.arm.goto(constants.armConsts.intakeAngle)
            )
        )
        # self.driverControler.x().negate().whileTrue(
        #     commands2.cmd.run(
        #         lambda: self.arm.goto(constants.shootingConsts.speakerPosition)
        #     )
        # )
        # self.driverControler.y().whileTrue(
        #     commands2.cmd.run(
        #         lambda: self.arm.intakeOVeride()
        #     )
        # ).whileFalse(
        #     lambda: self.arm.intake.set(0)
        # )
        # self.driverControler.rightTrigger().whileTrue(
        #     commands2.cmd.run(
        #         lambda: self.arm.pewpew()
        #     )
        # )

        # Right trigger 
        # While the right trigger is held it will spin up the shooter
        # and move the arm to the speaker position if not already there...
        self.driverControler.rightTrigger().whileTrue(
            commands2.cmd.run(
                spinUpShooters
            ).alongWith(
                commands2.cmd.run(
                    lambda: self.arm.goto(constants.shootingConsts.speakerPosition)
                )
            )
            # commands2.cmd.SequentialCommandGroup(
            #     commands2.cmd.run(lambda: self.arm.spinUpShooters()),
            #     commands2.cmd.waitSeconds(1),
            #     commands2.cmd.run(lambda: self.arm.shoot()),
            #     commands2.cmd.waitSeconds(1),
            #     commands2.cmd.run(lambda: self.arm.disableShooter())
            # )
        )

        # Right trigger when released
        # This will turn on the intake delivering the note to the shooter
        # Wait 1 second and then stop the shooter
        self.driverControler.rightTrigger().negate().whileTrue(
            commands2.cmd.SequentialCommandGroup(
                self.shootNoteObject,
                commands2.cmd.waitSeconds(1),
                self.stopShooterObject
            )
        )

        # Left Trigger
        # While the left trigger is held it will spin up the shooter
        # and move the arm to the amp scoring position if not already there
        self.driverControler.leftTrigger().whileTrue(
            commands2.cmd.run(
                spinUpShootersSlow
            ).alongWith(
                commands2.cmd.run(
                    lambda: self.arm.goto(constants.shootingConsts.ampPositon)
                )
            )
            # commands2.cmd.SequentialCommandGroup(
            #     commands2.cmd.run(lambda: self.arm.spinUpShooters()),
            #     commands2.cmd.waitSeconds(1),
            #     commands2.cmd.run(lambda: self.arm.shoot()),
            #     commands2.cmd.waitSeconds(1),
            #     commands2.cmd.run(lambda: self.arm.disableShooter())
            # )
        )

        # Left trigger when released
        # This will turn on the intake delivering the note to the shooter
        # Wait 1 second and then stop the shooter
        self.driverControler.leftTrigger().negate().whileTrue(
            commands2.cmd.SequentialCommandGroup(
                self.shootNoteObjectSlow,
                commands2.cmd.waitSeconds(1),
                self.stopShooterObjectSlow
            )
        )

        # Start button
        # Currently testing auto modes. 
        # For DEBUG purposes only
        self.driverControler.start().whileTrue(
            autoDriveForward(self.robotDrive)
        )
        # .whileFalse(
        #     commands2.cmd.run(
        #         lambda: self.arm.disableShooter()
        #     )
        # )

        self.driverControler.povUp().whileTrue(
            commands2.cmd.run(
                lambda: commands2.CommandScheduler.getInstance().cancelAll()
            )
        )

    def rumbleON(self):
        self.wpiXboxController.setRumble(XboxController.RumbleType.kBothRumble, 0.5)

    def rumbleOFF(self):
        self.wpiXboxController.setRumble(XboxController.RumbleType.kBothRumble, 0)
        
    def MoveArmToZeroAndReset(self):
        moveCmd = commands2.cmd.run(
            print("would be running arm @ 30%")#self.arm.set(0.3)
            ).until(lambda: self.arm.topLimit.get())
        
        
        # self.arm.arm.set(0.0)
        print(f"encoder pos right: {self.arm.armRightEncoderRelative.get()} | encoder pos left: {self.arm.armLeftEncoderRelative.get()}")
        self.arm.zeroEncodersRelative()
        print(f"encoder pos right: {self.arm.armRightEncoderRelative.get()} | encoder pos left: {self.arm.armLeftEncoderRelative.get()}")
        print(f"abs encoder pos right: {self.arm.armRightEncoder.getAbsolutePosition()} | abs encoder pos left: {self.arm.armLeftEncoder.getAbsolutePosition()}")


    def go_slow(self):
        self.scale_factor = constants.drivetrain.slowSpeed
        # print(self.scale_factor)

    def go_fast(self):
        self.scale_factor = 1
        # print(self.scale_factor)

    def toggleSlow(self):
        if self.isSlow:
            self.scale_factor = 1
            self.isSlow = False
        else:
            self.scale_factor = constants.drivetrain.slowSpeed
            self.isSlow = True
        # print(f"toggleSlow() - {self.isSlow}")

    def autonomousArmCommand(self):
        print("robotContainer.autonomousArmCommand()")
        # self.arm.armTargetAngle = constants.shootingConsts.speakerPosition
        self.arm.goto(constants.shootingConsts.speakerPosition)

    def getAutonomousArmCommand(self):
        return commands2.cmd.run(
            lambda: self.autonomousArmCommand()
        )
    
    def getAutoShootingCommand(self):
        return commands2.cmd.SequentialCommandGroup(
            commands2.cmd.run(
                lambda: self.arm.spinUpShooters()
            ).repeatedly().withTimeout(1),
            ShootNote(self.arm).repeatedly().withTimeout(1),
            StopShooter(self.arm).repeatedly().withTimeout(0.1)
        )

    def getAutoDriveCommand(self):
        print("getAutoDriveCommand")
        return commands2.cmd.run(
            lambda: self.robotDrive.drive(constants.autoConsts.driveSpeed, 0)
        )
    
    def getAutoReverseDriveCommand(self):
        print("getAutoReverseDriveCommand")
        return commands2.cmd.run(
            lambda: self.robotDrive.drive(-1.0 * constants.autoConsts.driveSpeed, 0)
        )

    def getAutoStopDriveCommand(self):
        print("getAutoStopDriveCommand")
        return commands2.cmd.run(
            lambda: self.robotDrive.drive(0, 0)
        )
    
    