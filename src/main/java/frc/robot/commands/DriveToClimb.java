package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.AutoConstants.*;

/**
 * Autonomous routine: wait for a configurable delay, drive straight for a
 * configurable duration, then climb.
 */
public class DriveToClimb extends SequentialCommandGroup {

  public DriveToClimb(CANDriveSubsystem driveSubsystem, ClimberSubsystem climberSubsystem) {
    addCommands(
        // Wait before doing anything
        new WaitCommand(DRIVE_TO_CLIMB_WAIT_SECONDS),
        // Drive straight for the configured time
        new AutoDrive(driveSubsystem, DRIVE_TO_CLIMB_SPEED, 0.0)
            .withTimeout(DRIVE_TO_CLIMB_DRIVE_SECONDS),
        // Climb up (ClimbUp ends automatically when position > 60)
        new ClimbUp(climberSubsystem));
  }
}
