from typing import Any, Callable
from commands2.wrappercommand import WrapperCommand
from subSystems.armSubsystem import ArmSubsystem
import commands2
from wpilib import XboxController

class DetectNote(commands2.Command):
    def __init__(self, arm: ArmSubsystem, xboxcontroller=None):
        self.arm = arm
        self.xboxcontroller = xboxcontroller

    def execute(self):
        # wait for sensor
        pass

    def isFinished(self) -> bool:
        return self.arm.isNoteLoaded()
    
    def handleInterrupt(self, handler: Callable[[], Any]) -> WrapperCommand:
        if self.xboxcontroller is not None:
            self.xboxcontroller.setRumble(XboxController.RumbleType.kBothRumble, 0)
        return super().handleInterrupt(handler)
    
    def end(self, interrupted):
        if self.xboxcontroller is not None:
            self.xboxcontroller.setRumble(XboxController.RumbleType.kBothRumble, 0)
    



