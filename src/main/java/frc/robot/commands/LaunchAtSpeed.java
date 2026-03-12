package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;

/**
 * Launches at a specific roller speed for distance control.
 * Sets the launcher rollers and feeder to the given speeds immediately.
 */
public class LaunchAtSpeed extends Command {

  private final CANFuelSubsystem fuelSubsystem;
  private final double launcherSpeed;
  private final double feederSpeed;

  /**
   * @param fuelSubsystem the fuel subsystem
   * @param launcherSpeed the speed for the launcher rollers (negative = launch direction)
   * @param feederSpeed   the speed for the jiggler/feeder roller
   */
  public LaunchAtSpeed(CANFuelSubsystem fuelSubsystem, double launcherSpeed,
      double feederSpeed) {
    this.fuelSubsystem = fuelSubsystem;
    this.launcherSpeed = launcherSpeed;
    this.feederSpeed = feederSpeed;
    addRequirements(fuelSubsystem);
  }

  @Override
  public void initialize() {
    fuelSubsystem.setIntakeLauncherRoller(launcherSpeed);
    fuelSubsystem.setFeederRoller(feederSpeed);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
