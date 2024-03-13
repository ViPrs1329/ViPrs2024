import commands2
import commands2.button
import commands2.cmd
import numpy as DrArnett

from subSystems.REVDriveSubsystem import DriveSubsystem
from subSystems.armSubsystem import ArmSubsystem
from commands.shootNote import ShootNote
from commands.stopShooter import StopShooter
from commands.retractNote import Backup
from commands.pickupNote import PickupNote
from commands.detectNote import DetectNote

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
        self.robotDrive = DriveSubsystem()
        self.arm = ArmSubsystem()
        self.shootNoteObject = ShootNote(self.arm)
        self.stopShooterObject = StopShooter(self.arm)
        self.backupObject = Backup(self.arm)
        self.pickupObject = PickupNote(self.arm)
        self.detectNoteObject = DetectNote(self.arm)
        self.configureButtonBindings()
        
        self.scale_factor = 1

        print(self.shootNoteObject)

        self.robotDrive.setDefaultCommand(
            commands2.cmd.run(
                lambda: self.robotDrive.robotDrive.arcadeDrive(
                    shapeInputs(
                        -self.driverControler.getLeftY(), self.scale_factor
                    ),
                    shapeInputs(
                        -self.driverControler.getRightX(), self.scale_factor
                    )
                ),
                self.robotDrive
            ).alongWith(
                commands2.cmd.run(
                    lambda: self.arm.updateArmPosition()
                )
            )
            # .alongWith(
            #     commands2.cmd.run(
            #         lambda: self.arm.shooterIdle()
            #     )
            # )
            
        )

    def configureButtonBindings(self):
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

        self.driverControler.a().whileTrue(
            commands2.cmd.run(
                lambda: self.arm.goto(0.8)
            )
        ).whileFalse(
            commands2.cmd.run(
                lambda: self.arm.goto(0.0)
            )
        )

        self.driverControler.x().whileTrue(
            commands2.cmd.SequentialCommandGroup(
                self.pickupObject,
                self.detectNoteObject,
                self.backupObject
            )
        ).negate().whileTrue(
            commands2.cmd.run(
                lambda: self.arm.intake.set(0)
            )
        )
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
            
        def spinUpShooters():
            self.arm.spinUpShooters()

        self.driverControler.rightTrigger().whileTrue(
            commands2.cmd.run(
                spinUpShooters
            )
            # commands2.cmd.SequentialCommandGroup(
            #     commands2.cmd.run(lambda: self.arm.spinUpShooters()),
            #     commands2.cmd.waitSeconds(1),
            #     commands2.cmd.run(lambda: self.arm.shoot()),
            #     commands2.cmd.waitSeconds(1),
            #     commands2.cmd.run(lambda: self.arm.disableShooter())
            # )
        )
        self.driverControler.rightTrigger().negate().whileTrue(
            commands2.cmd.SequentialCommandGroup(
                self.shootNoteObject,
                commands2.cmd.waitSeconds(1),
                self.stopShooterObject
            )
        )
        # .whileFalse(
        #     commands2.cmd.run(
        #         lambda: self.arm.disableShooter()
        #     )
        # )
        # self.driverControler.y().whileTrue(
        #     commands2.cmd.sequence(
        #         lambda: print("trigger"),
        #         lambda: self.arm.spinUpShooters(),
        #         commands2.cmd.waitSeconds(1),
        #         lambda: self.arm.shoot(),
        #         commands2.cmd.waitSeconds(1),
        #         lambda: self.arm.disableShooter()
        #     )
        # ).whileFalse(
        #     commands2.cmd.run(
        #         lambda: self.arm.disableShooter()
        #     )
        # )
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
        self.scale_factor = 0.5
        print(self.scale_factor)

    def go_fast(self):
        self.scale_factor = 1
        print(self.scale_factor)

    def getAutonomousCommand(self):
        return commands2.cmd.none()
    
