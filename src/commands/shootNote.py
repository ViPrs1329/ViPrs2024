import commands2
from subSystems.armSubsystem import ArmSubsystem

import constants

class ShootNote(commands2.Command):
    def __init__(self, arm: ArmSubsystem):
        super().__init__()
        self.arm = arm

    def initialize(self):
        self.arm.shoot()

    def isFinished(self) -> bool:
        return True