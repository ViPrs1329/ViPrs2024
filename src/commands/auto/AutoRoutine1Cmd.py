from commands2 import SequentialCommandGroup
from commands.drive.driveForDistanceCmd import DriveForDistanceCmd
from commands.drive.rotateToAngleCmd import RotateToAngle
from commands.arm.MoveArmCmds import MoveArmToScoreHigh
from commands.shooter.shooterStartCmd import ShooterStartCmd
from commands.shooter.shooterStopCmd import ShooterStopCmd
from commands2.cmd import waitSeconds

from subSystems.REVDriveSubsystem import DriveSubsystem
from subSystems.armSubsystem import ArmSubsystem
from subSystems.shooterSubsystem import ShooterSubsystem

class AutonomousRoutine(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem, arm: ArmSubsystem, shooter: ShooterSubsystem):
        super().__init__(
            # Move 36 inches forward
            DriveForDistanceCmd(drive, 36),
            # Rotate down 45 degrees
            RotateToAngle(drive, -45),
            # Move 39 inches forward
            DriveForDistanceCmd(drive, 39),
            # Raise arm to the high shooting position
            MoveArmToScoreHigh(arm),
            # Start shooting
            ShooterStartCmd(shooter),
            # Assuming we want to shoot for a certain duration before stopping
            waitSeconds(2),  # Wait for 2 seconds (adjust time as needed)
            ShooterStopCmd(shooter),
            # Move back 39 inches
            DriveForDistanceCmd(drive, -39),
            # Rotate back to original orientation (45 degrees up from current position)
            RotateToAngleS(drive, 45),
            # Move 108 inches forward
            DriveForDistanceCmd(drive, 108),
            # No need for a stop command if DriveForDistanceCmd and RotateToAngleCmd stop the robot on completion
        )
