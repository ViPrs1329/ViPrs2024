import commands2
from subSystems.armSubsystem import ArmSubsystem
import constants
class GotoTaxiPosition(commands2.Command):
    def __init__(self, arm: ArmSubsystem):
        super().__init__()
        self.arm = arm

    def initialize(self):
        self.arm.goto(constants.shootingConsts.speakerPosition)

    def isFinished(self):
        return True