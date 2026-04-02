package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.AutoConstants.*;

/**
 * Autonomous routine: climb down first, wait for a configurable delay,
 * drive straight for a configurable duration, then climb back up.
 */
public class DriveToClimb extends SequentialCommandGroup {

  public DriveToClimb(CANDriveSubsystem driveSubsystem, ClimberSubsystem climberSubsystem) {
    addCommands(
        // Lower the climber first
        new ClimbUp(climberSubsystem),
        // Wait before driving
        new WaitCommand(DRIVE_TO_CLIMB_WAIT_SECONDS),
        // Drive straight for the configured time
        new AutoDrive(driveSubsystem, DRIVE_TO_CLIMB_SPEED, 0.0)
            .withTimeout(DRIVE_TO_CLIMB_DRIVE_SECONDS),
        // Climb up (ClimbUp ends automatically when position > 60)
        new ClimbDown(climberSubsystem));
  }
}
