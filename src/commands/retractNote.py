import commands2
from subSystems.armSubsystem import ArmSubsystem

import constants

class Backup(commands2.Command):
    def __init__(self, arm: ArmSubsystem):
        super().__init__()
        self.arm = arm
        self.ini

    def initialize(self):
        self.arm.intake.set(constants.shootingConsts.backupSpeed)
        
    def isFinished(self) -> bool:
        return True