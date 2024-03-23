import commands2
from subSystems.armSubsystem import ArmSubsystem
import constants
class Unschedule(commands2.Command):
    def __init__(self, arm: ArmSubsystem, command):
        super().__init__()
        self.arm = arm
        self.command = command

    def initialize(self):
        commands2.CommandScheduler.getInstance().cancel(self.command)

    def isFinished(self):
        return True