import commands2.button
from commands2 import SequentialCommandGroup
from commands2.button import CommandXboxController

# Import subsystems
from subSystems.REVDriveSubsystem import DriveSubsystem
from subSystems.armSubsystem import ArmSubsystem
from subSystems.intakeSubsystem import IntakeSubsystem
from subSystems.shooterSubsystem import ShooterSubsystem

# Import commands
from commands.intake.intakeCollectNoteCmd import IntakeCollectNoteCmd
from commands.intake.intakeRetractNoteCmd import IntakeRetractNoteCmd
from commands.intake.intakeAndRetractCmd import IntakeAndRetractCommand
from commands.intake.intakeStartCmd import IntakeStartCmd
from commands.intake.detectNoteCmd import DetectNoteCmd
from commands.intake.stopIntakeCmd import StopIntakeCmd
from commands.intake.intakeExpelNoteCmd import IntakeExpelNoteCmd
from commands.intake.intakeDeliverNoteToShooterCmd import IntakeDeliverNoteToShooterCmd
from commands.shooter.shooterStartCmd import ShooterStartCmd
from commands.shooter.shooterStopCmd import ShooterStopCmd
from commands.arm.MoveArmCmds import MoveArmToIntakePosition, MoveArmToScoreHigh, MoveArmToScoreLow, MoveArmToStartingPosition
from commands.drive.arcadeDriveCmd import ArcadeDriveCmd
from commands.drive.driveSlowCmd import ToggleSlowModeCmd
from commands.drive.toggleReverseDriveCmd import ToggleReverseDriveCmd
from commands.auto.AutoRoutine1Cmd import AutonomousRoutine

# Import constants
from constants import States, intakeConsts, inputConsts, driveConsts
from subSystems.robotState import RobotState

class RobotContainer:
    """
    This class is where the bulk of the robot's resources are declared. Here, subsystems
    are instantiated and commands and button bindings are configured.
    """
    def __init__(self):
        self.initSubsystems()
        self.initControls()
        self.initCommands()
        self.configureButtonBindings()

    def initSubsystems(self):
        """Instantiate the robot's subsystems."""
        self.robotDrive = DriveSubsystem()
        self.arm = ArmSubsystem()
        self.intake = IntakeSubsystem()
        self.shooter = ShooterSubsystem()
        # Configurations for subsystems
        self.robotDrive.setMaxOutput(driveConsts.driveMaxOutput)

    def initControls(self):
        """Instantiate the robot's control objects"""
        self.driverController = commands2.button.CommandXboxController(0)
        self.rightTriggerPressed = self.driverController.rightTrigger(threshold=0.5)
        self.leftTriggerPressed = commands2.button.Trigger(lambda: self.driverController.getLeftTriggerAxis() > 0.5)

    def initCommands(self):
        """Instantiate the robot's commands."""
        # Arm commands
        self.moveToStartingPosition = MoveArmToStartingPosition(self.arm)
        self.moveToScoreHigh = MoveArmToScoreHigh(self.arm)
        self.moveToScoreLow = MoveArmToScoreLow(self.arm)
        self.moveToIntakePosition = MoveArmToIntakePosition(self.arm)

        # Intake commands
        self.collectCmd = IntakeCollectNoteCmd(self.intake, intakeConsts.captureSpeed)
        self.retractCmd = IntakeRetractNoteCmd(self.intake, intakeConsts.releaseSpeed, intakeConsts.retractTime)
        self.intakeAndRetractCommand = IntakeAndRetractCommand(self.intake, intakeConsts.releaseSpeed, intakeConsts.retractTime)
        self.intakeStartCmd = IntakeStartCmd(self.intake, intakeConsts.captureSpeed)
        self.stopIntakeCmd = StopIntakeCmd(self.intake)
        self.detectNoteCmd = DetectNoteCmd(self.intake)
        self.expelNoteCmd = IntakeExpelNoteCmd(self.intake, intakeConsts.expelSpeed, intakeConsts.expelTime)
        self.deliverNoteCmd = IntakeDeliverNoteToShooterCmd(self.intake)

        # Shooter commands
        self.startShooterCmd = ShooterStartCmd(self.shooter)
        self.stopShooterCmd = ShooterStopCmd(self.shooter)

        # Drive commands
        self.arcadeDriveCmd = ArcadeDriveCmd(self.robotDrive, self.driverController)
        self.driveSlowCmd = ToggleSlowModeCmd(self.robotDrive)
        self.reverseDriveCmd = ToggleReverseDriveCmd(self.robotDrive)

        # Command groups
        #self.collectAndRetractCmd = SequentialCommandGroup(self.collectCmd, self.retractCmd)
        self.intakeCommandGroup = SequentialCommandGroup(self.intakeStartCmd, self.detectNoteCmd, self.retractCmd)
        # self.intakeCommandGroup = SequentialCommandGroup(self.intakeStartCmd, self.detectNoteCmd, self.stopIntakeCmd)

        # Set up default commands for subsystems
        self.robotDrive.setDefaultCommand(self.arcadeDriveCmd)
        # self.arm.setDefaultCommand(???)
        # self.intake.setDefaultCommand(???)
        # self.shooter.setDefaultCommand(???)

    def configureButtonBindings(self):
        """Configure the button bindings for user input."""
               
        # Button A binding - Move arm to intake position
        self.driverController.a().onTrue(self.moveToIntakePosition)
        
        # Button B binding
        # self.driverController.b().toggleOnTrue(self.collectAndRetractCmd)
        # self.driverController.b().toggleOnTrue(self.collectCmd)  # Test simple intake
        # self.driverController.b().onTrue(IntakeAndRetractCommand(self.intake, -0.5, 2.0))
        self.driverController.b().onTrue(self.intakeCommandGroup)

        # Button X binding
        self.driverController.x().onTrue(self.expelNoteCmd)

        # Button Y binding
        self.driverController.y().onTrue(
            AutonomousRoutine(self.robotDrive, self.arm, self.shooter, self.intake)
        )

        # Right bumper binding
        # Move arm to speaker (high) scoring position
        self.driverController.rightBumper().onTrue(self.moveToScoreHigh)

        # Left bumper binding
        # Move arm to amp (low) scoring position
        self.driverController.leftBumper().onTrue(self.moveToScoreLow)

        # Right trigger binding 
        self.rightTriggerPressed.whileTrue(self.startShooterCmd)
        self.rightTriggerPressed.negate().whileTrue(SequentialCommandGroup(
            self.deliverNoteCmd,
            self.stopShooterCmd,
            self.stopIntakeCmd
        ))

        # Left trigger binding
        self.leftTriggerPressed.whileTrue(self.driveSlowCmd)

        # Start button binding

        # "Back" button binding. (This looksl ike the menu button)
        self.driverController.back().onTrue(self.reverseDriveCmd)

    def updateHardware(self):
        """Call the update methods of each subsystem."""
        self.robotDrive.updateHardware()
        self.arm.updateHardware()
        self.intake.updateHardware()
        self.shooter.updateHardware()

    def cacheSensors(self):
        """Retrieve and cache sensor data from each subsystem."""
        self.robotDrive.cacheSensors()
        self.arm.cacheSensors()
        self.intake.cacheSensors()
        self.shooter.cacheSensors()

    def getAutonomousCommand(self):
        """Return the command to run in autonomous mode."""
        # Placeholder for an autonomous command. Modify as needed.
        return commands2.cmd.none()
