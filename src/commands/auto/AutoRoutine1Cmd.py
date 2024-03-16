from commands2 import SequentialCommandGroup, ParallelDeadlineGroup
from commands.drive.driveForDistanceCmd import DriveForDistanceCmd
from commands.drive.rotateToAngleCmd import RotateToAngle
from commands.arm.MoveArmCmds import MoveArmToScoreHigh
from commands.shooter.shooterStartCmd import ShooterStartCmd
from commands.shooter.shooterStopCmd import ShooterStopCmd
from commands.intake.intakeDeliverNoteToShooterCmd import IntakeDeliverNoteToShooterCmd
from commands2.cmd import waitSeconds

from subSystems.REVDriveSubsystem import DriveSubsystem
from subSystems.armSubsystem import ArmSubsystem
from subSystems.shooterSubsystem import ShooterSubsystem
from subSystems.intakeSubsystem import IntakeSubsystem

'''
# Attempt 3
class AutonomousRoutine(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem, arm: ArmSubsystem, shooter: ShooterSubsystem, intake: IntakeSubsystem):
        super().__init__(
            DriveForDistanceCmd(drive, 36),
            RotateToAngle(drive, -45),
            DriveForDistanceCmd(drive, 39),
            MoveArmToScoreHigh(arm),  # Raise arm to scoring position.
            waitSeconds(2),
            ShooterStartCmd(shooter),
            waitSeconds(2),  # Assuming some time for shooting; adjust based on your needs.
            IntakeDeliverNoteToShooterCmd(intake),
            ShooterStopCmd(shooter),
            # DriveForDistanceCmd(drive, -39),
            # RotateToAngle(drive, 45),
            # DriveForDistanceCmd(drive, -108)
        )
'''

'''
# Attempt 2 (This gets stuck on ShooterStartCmd as that command never finishes)
class AutonomousRoutine(SequentialCommandGroup):
    def __init__(self, drive: DriveSubsystem, arm: ArmSubsystem, shooter: ShooterSubsystem, intake: IntakeSubsystem):
        super().__init__(
            DriveForDistanceCmd(drive, 36),
            RotateToAngle(drive, -45),
            DriveForDistanceCmd(drive, 39),
            ParallelDeadlineGroup(
                SequentialCommandGroup(
                    ShooterStartCmd(shooter),
                    waitSeconds(2),
                    IntakeDeliverNoteToShooterCmd(intake),
                    ShooterStopCmd(shooter)
                ),
                MoveArmToScoreHigh(arm)  # This command runs in parallel but doesn't dictate the end of the group.
            )
            # DriveForDistanceCmd(drive, -39),
            # RotateToAngle(drive, 45),
            # DriveForDistanceCmd(drive, -108)
        )
'''
        
class AutonomousRoutine(ParallelDeadlineGroup):
    def __init__(self, drive: DriveSubsystem, arm: ArmSubsystem, shooter: ShooterSubsystem, intake: IntakeSubsystem):
        super().__init__(
            MoveArmToScoreHigh(arm), # This is the deadline command
            SequentialCommandGroup(
                DriveForDistanceCmd(drive, 36),
                RotateToAngle(drive, -45),
                DriveForDistanceCmd(drive, 39),
                ShooterStartCmd(shooter),
                waitSeconds(2),
                IntakeDeliverNoteToShooterCmd(intake),
                ShooterStopCmd(shooter),
                DriveForDistanceCmd(drive, -39),
                RotateToAngle(drive, 45),
                DriveForDistanceCmd(drive, -108)
            )
        )
    

''' Attempt 1, gets stuck on MoveArmToScoreHigh(arm) command
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
            RotateToAngle(drive, 45),
            # Move 108 inches forward
            DriveForDistanceCmd(drive, -108),
            # No need for a stop command if DriveForDistanceCmd and RotateToAngleCmd stop the robot on completion
        )
'''