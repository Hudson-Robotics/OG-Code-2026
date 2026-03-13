package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

import static frc.robot.Constants.AutoConstants.*;

/**
 * Autonomous routine: drive forward, then spin up and launch.
 * Like ShootAndClimb but without the climb.
 */
public class JustShoot extends SequentialCommandGroup {

  public JustShoot(CANDriveSubsystem driveSubsystem, CANFuelSubsystem fuelSubsystem) {
    addCommands(
        // Drive forward first
        new AutoDrive(driveSubsystem, JUST_SHOOT_DRIVE_SPEED, 0.0)
            .withTimeout(JUST_SHOOT_DRIVE_SECONDS),
        // Spin up the launcher wheels
        new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        // Launch for the configured time
        new Launch(fuelSubsystem).withTimeout(JUST_SHOOT_SECONDS));
  }
}
