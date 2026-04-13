// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Single command that handles the full flywheel launch cycle:
 * 1. Spins the flywheel up to the target RPS using closed-loop velocity control.
 * 2. Once at speed, automatically turns on the feeder to launch the ball.
 * 3. Keeps the flywheel running in closed-loop the entire time so it recovers
 *    after the ball impacts the rollers.
 *
 * Runs until the button is released (whileTrue binding).
 */
public class FlywheelLaunchSequence extends Command {

  private final CANFuelSubsystem fuelSubsystem;
  private final double targetRPS;

  /**
   * @param fuelSubsystem the fuel subsystem
   * @param targetRPS     desired flywheel velocity in RPS (negative = launch direction)
   */
  public FlywheelLaunchSequence(CANFuelSubsystem fuelSubsystem, double targetRPS) {
    this.fuelSubsystem = fuelSubsystem;
    this.targetRPS = targetRPS;
    addRequirements(fuelSubsystem);
  }

  /** Convenience constructor using the default launch RPS from Constants. */
  public FlywheelLaunchSequence(CANFuelSubsystem fuelSubsystem) {
    this(fuelSubsystem, FuelConstants.DEFAULT_LAUNCH_RPS);
  }

  @Override
  public void initialize() {
    // Start spinning the flywheel to the target velocity (closed-loop)
    fuelSubsystem.setFlywheelVelocity(targetRPS);
    // Hold the feeder still while the flywheel spins up
    fuelSubsystem.setFeederRoller(FLYWHEEL_SPINUP_FEEDER_PERCENT);
  }

  @Override
  public void execute() {
    // Keep commanding the flywheel every loop
    fuelSubsystem.setFlywheelVelocity(targetRPS);

    // Once at speed, feed the ball; otherwise keep holding
    if (fuelSubsystem.isAtTargetVelocity()) {
      fuelSubsystem.setFeederRoller(INDEXER_LAUNCHING_PERCENT);
    } else {
      fuelSubsystem.setFeederRoller(FLYWHEEL_SPINUP_FEEDER_PERCENT);
    }
  }

  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // Runs until the button is released
    return false;
  }
}
