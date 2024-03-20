import commands2
from subSystems.armSubsystem import ArmSubsystem

import constants

from wpilib import XboxController

class PickupNote(commands2.Command):
    def __init__(self, arm: ArmSubsystem, xboxcontroller=None):
        super().__init__()
        self.arm = arm
        self.xboxcontroller = xboxcontroller

    def initialize(self):
        self.arm.pickup()
        if self.xboxcontroller is not None:
            self.xboxcontroller.setRumble(XboxController.RumbleType.kBothRumble, 0.5)

    def isFinished(self) -> bool:
        return True