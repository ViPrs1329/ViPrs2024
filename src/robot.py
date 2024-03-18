# TODO: insert robot code here
#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import wpilib.drive
import phoenix5
import rev
import commands2
from robotContainer import RobotContainer
from commands.pickupNote import PickupNote
from commands.detectNote import DetectNote
from commands.retractNote import Backup
import constants

class MyRobot(commands2.TimedCommandRobot):
    autonomousCommand = None
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        # self.timer = wpilib.Timer()

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.container = RobotContainer()
        
        # Is this what we want at the beginning? It sets arm.isActive to True,
        # But it also sets the Idle Mode of the arm speed controllers to
        # Coast mode
        self.container.arm.activate()  

    def robotPeriodic(self):
        self.container.arm.updateArmPosition() 
        self.container.robotDrive.updateDrive()
        commands2.CommandScheduler.getInstance().run()
    
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        # self.timer.restart()

        # Set default command (which should be sending 0, 0 to ArcadeDrive)
        # self.container.setAutoDefaultCommand()
        
        # self.autonomousArmCommand = self.container.getAutonomousArmCommand()
        # self.autonomousArmCommand.schedule()
        # if self.autonomousArmCommand:
        #     self.autonomousArmCommand.schedule()

        # self.autoDrive = self.container.getAutoDriveCommand()
        # self.autoDrive.schedule()

        # self.autoShooting = self.container.getAutoShootingCommand()
        # self.autoShooting.schedule()

        # self.autoReverseDrive = self.container.getAutoReverseDriveCommand()
        # self.autoReverseDrive.schedule()

        '''
        Steps for Auto:
        - At the same time:
            - Raise arm to shoot high
            - Drive forward for a certain amount of time (see consts)
        - Then, stop drive
        - Perform shooting sequence:
            - Spin up shooters
            - Shoot note
            - Stop shooters
        - Then drive in reverse out of the auto zone for a certain amount of time
        - Then stop drive

        Simple auto:
        - Raise arm to shoot high
        - Start shooting sequence
        - Drive out of zone for period of time (Time TBD)
        - Stop drive

        
        Moonshot additions:
        - When driving out of the zone deploy arm for pickup
        - Intake drive back to subwoofer shoot
        - Move back out of the starting zone
        '''

        '''
        self.autonomousCommand = commands2.SequentialCommandGroup(
            commands2.ParallelCommandGroup(
                commands2.SequentialCommandGroup(
                    self.container.getAutoReverseDriveCommand().repeatedly().withTimeout(1),
                    self.container.getAutoStopDriveCommand().repeatedly().withTimeout(0.1),
                ),
                self.container.getAutonomousArmCommand().repeatedly().withTimeout(0.1)
            ),    
            self.container.getAutoShootingCommand(),        
            self.container.getAutoDriveCommand().repeatedly().withTimeout(1),
            self.container.getAutoStopDriveCommand().repeatedly().withTimeout(0.1)
        )
        '''

        # Simplified auto, since we can start right next to the subwoofer
        self.autonomousCommand = commands2.SequentialCommandGroup(
            self.container.getAutonomousArmCommand().repeatedly().withTimeout(0.5),
            self.container.getAutoShootingCommand(),        
            self.container.getAutoDriveCommand().repeatedly().withTimeout(constants.autoConsts.simpleDriveTimeOutOfZone),
            self.container.getAutoStopDriveCommand().repeatedly().withTimeout(0.1)
        )

        # Cherry on top if we can get this
        '''
        - Raise arm to speaker shooting angle
        - Shoot
        - Bring arm down to intake collecting angle while starting intake sequence
        - Drive forward for 3* seconds
        - Stop drive briefly
        - Drive backward for 3* seconds
        - Stop drive
        - Raise arm to speaker shooting angle
        - Shoot high
        - Drive forward for 3* seconds
        - Stop drive
        '''
        self.megaAutonomousCommand = commands2.SequentialCommandGroup(
            self.container.getAutonomousArmCommand().repeatedly().withTimeout(0.5),
            self.container.getAutoShootingCommand(), 
            commands2.ParallelCommandGroup(
                commands2.cmd.run(
                lambda: self.container.arm.goto(constants.armConsts.intakeAngle)
                ).repeatedly().withTimeout(0.5),
                commands2.cmd.SequentialCommandGroup(
                    PickupNote(self.container.arm),
                    DetectNote(self.container.arm),
                    Backup(self.container.arm)
                )
            ),
            self.container.getAutoDriveCommand().repeatedly().withTimeout(constants.autoConsts.driveTimeSubToMiddleNote),
            self.container.getAutoStopDriveCommand().repeatedly().withTimeout(0.1),
            self.container.getAutoReverseDriveCommand().repeatedly().withTimeout(constants.autoConsts.driveTimeSubToMiddleNote),
            self.container.getAutoStopDriveCommand().repeatedly().withTimeout(0.1),
            self.container.getAutonomousArmCommand().repeatedly().withTimeout(0.5),
            self.container.getAutoShootingCommand(), 
            self.container.getAutoDriveCommand().repeatedly().withTimeout(constants.autoConsts.driveTimeSubToMiddleNote),
            self.container.getAutoStopDriveCommand().repeatedly().withTimeout(0.1)
        )

        # This is a very simple auto. Just raise arm and shoot.
        # Useful for when your alliance has a really talented auto 
        self.verySimpleAutoCommand = commands2.SequentialCommandGroup(
            self.container.getAutonomousArmCommand().repeatedly().withTimeout(0.5),
            self.container.getAutoShootingCommand()
        )

        self.autonomousCommand.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        pass

    def disabledInit(self):
        # self.container.arm.goto(constants.shootingConsts.safePosition)
        commands2.CommandScheduler.getInstance().cancelAll()
        print("**DISABLED!**")

    def disabledPeriodic(self):
        pass

    def teleopInit(self): 
        """This function is called once each time the robot enters teleoperated mode."""
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

        # This is not how you set the default command
        # self.teleopDefaultCommand = self.container.setTeleopDefaultCommand()
        # self.teleopDefaultCommand.schedule()
            
        self.container.setTeleopDefaultCommand()
        
    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        # print(self.container.arm.getArmPosition())  # check to see if they are zero'd
        # print(self.container.arm)

    def testInit(self): 
        """This function is called once each time the robot enters test mode."""
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.arm.zeroEncoders()
        self.container.MoveArmToZeroAndReset()
        
    def testPeriodic(self): 
        """This function is called periodically during test mode."""
        print(self.container.arm)

if __name__ == "__main__":
    wpilib.run(MyRobot)



