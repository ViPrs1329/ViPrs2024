import commands2
import wpilib
from subSystems.armSubsystem import ArmSubsystem

import constants

class Backup(commands2.Command):
    def __init__(self, arm: ArmSubsystem):
        super().__init__()
        self.arm = arm
        self.initialTime = 0

    def initialize(self):
        self.initialTime = wpilib.Timer.getFPGATimestamp()

    def execute(self):
        self.arm.intake.set(constants.shootingConsts.backupSpeed)
        
    def isFinished(self) -> bool:
        elapsedTime = wpilib.Timer.getFPGATimestamp() - self.initialTime
        return elapsedTime >= constants.shootingConsts.backupTime
    
    def end(self, interrupted):
        self.arm.intake.set(0)