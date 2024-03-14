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

    def robotPeriodic(self):
        self.container.arm.updateArmPosition()
        commands2.CommandScheduler.getInstance().run()
    
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        # self.timer.restart()
        self.autonomousCommand = self.container.getAutonomousCommand()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        pass

    def disabledInit(self):
        # self.container.arm.goto(constants.shootingConsts.safePosition)
        print("**DISABLED!**")

    def disabledPeriodic(self):
        pass

    def teleopInit(self): 
        """This function is called once each time the robot enters teleoperated mode."""
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
        
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