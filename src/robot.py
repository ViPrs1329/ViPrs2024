# TODO: insert robot code here
#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import time
import wpilib
import wpilib.drive
import rev
import commands2
from ntcore import NetworkTableInstance
from robotContainer import RobotContainer
import constants
from team1329.SimplePIDController import SimplePIDController
from team4646.PID import PID

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

        # Keep arm at starting position at the start of the match. Can't 
        # put this as a default comamnd for the arm subsystem because it
        # would get called when no arm position command is running
        # self.moveArmToStart = self.container.moveToStartingPosition
        # self.moveArmToStart.schedule()

        self.ntinst = NetworkTableInstance.getDefault()
        self.ntinst.startServer()

        # Simulation code
        if wpilib.RobotBase.isSimulation():
            self.simulationInit()

    def robotPeriodic(self):
        # print("robotPeriodic()")
        self.container.cacheSensors()
        commands2.CommandScheduler.getInstance().run()
        self.container.updateHardware()

        if wpilib.RobotBase.isSimulation():
            self.SimulationPeriodic()
    
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        # print("autonomousInit()")
        # self.timer.restart()
        self.autonomousCommand = self.container.getAutonomousCommand()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        # print("autonomousPeriodic()")

        pass

    def disabledInit(self):
        self.container.arm.isActive = False
        # robotContainer.onDisable()
        print("**DISABLED!**")

    def disabledPeriodic(self):
        pass

    def teleopInit(self): 
        """This function is called once each time the robot enters teleoperated mode."""
        print("teleopInit()")
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
        
    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        # print("teleopPeriodic()")
        # print(self.container.arm)

    def testInit(self): 
        """This function is called once each time the robot enters test mode."""
        print("testInit()")
        commands2.CommandScheduler.getInstance().cancelAll()
        #self.container.arm.zeroEncoders()
        #self.container.MoveArmToZeroAndReset()
        
    def testPeriodic(self): 
        """This function is called periodically during test mode."""
        # print("testPeriodic()")
        pass

    def simulationInit(self):
        print("Simulation init...")
        self.PIDTest = SimplePIDController(constants.armConsts.armPIDValues)
        self.PIDTestCurrentVal = 0
        self.lastUpdateTime = time.time()
        self.PIDTest.set_setpoint(1.5, 0.0)

    def SimulationPeriodic(self):
        """"This function is called periodically during the simulation mode"""
        # print("SimulationPeriodic()")
        now = time.time()
        dt = now - self.lastUpdateTime
        self.lastUpdateTime = now
        print(f"sim -> PID: {self.PIDTest.update(self.PIDTestCurrentVal, dt, 0.0)}")


if __name__ == "__main__":
    wpilib.run(MyRobot)