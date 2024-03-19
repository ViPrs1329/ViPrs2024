import commands2
import wpilib
# Can't import RobotContainer

import constants

class ToggleSlow(commands2.Command):
    def __init__(self, rc):  #
        super().__init__()
        self.rc = rc

    def initialize(self):
        self.rc.toggleSlow()

    def execute(self):
        pass
        
    def isFinished(self) -> bool:
        return True