package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.LaunchSequence;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.AutoConstants.*;

/**
 * Autonomous routine: climb up, drive, launch, then climb down — all sequential.
 */
public class ShootAndClimb extends SequentialCommandGroup {

  public ShootAndClimb(CANDriveSubsystem driveSubsystem, CANFuelSubsystem fuelSubsystem,
      ClimberSubsystem climberSubsystem) {
    addCommands(
        new ClimbUp(climberSubsystem),
        new AutoDrive(driveSubsystem, SHOOT_AND_CLIMB_DRIVE_SPEED, 0.0)
            .withTimeout(SHOOT_AND_CLIMB_DRIVE_SECONDS),
        new LaunchSequence(fuelSubsystem)
            .withTimeout(SHOOT_AND_CLIMB_SHOOT_SECONDS),
        new ClimbDown(climberSubsystem));
  }
}
