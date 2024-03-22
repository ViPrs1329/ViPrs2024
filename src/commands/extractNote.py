import commands2
import wpilib
from subSystems.armSubsystem import ArmSubsystem

import constants

class Extract(commands2.Command):
    def __init__(self, arm: ArmSubsystem):
        super().__init__()
        self.arm = arm
        self.initialTime = 0

    def initialize(self):
        self.initialTime = wpilib.Timer.getFPGATimestamp()

    def execute(self):
        self.arm.intake.set(constants.armConsts.extractionSpeed)
        
    def isFinished(self) -> bool:
        elapsedTime = wpilib.Timer.getFPGATimestamp() - self.initialTime
        return elapsedTime >= constants.armConsts.extractionTime
    
    def end(self, interrupted):
        self.arm.intake.set(0)