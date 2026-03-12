package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANFuelSubsystem;

import static frc.robot.Constants.AutoConstants.*;

/**
 * Autonomous routine: spin up, then launch for a configurable duration.
 * Does nothing else afterwards.
 */
public class JustShoot extends SequentialCommandGroup {

  public JustShoot(CANFuelSubsystem fuelSubsystem) {
    addCommands(
        // Spin up the launcher wheels first
        new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        // Launch for the configured time
        new Launch(fuelSubsystem).withTimeout(JUST_SHOOT_SECONDS));
  }
}
