// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Spins the flywheel (intake/launcher rollers) to a target velocity using
 * closed-loop control. Finishes automatically once the flywheel reaches the
 * target RPS — no more guessing with timeouts!
 *
 * While the flywheel is spinning up, the feeder runs at a slow hold-back speed
 * to keep the ball from feeding prematurely.
 */
public class SpinUpFlywheel extends Command {

  private final CANFuelSubsystem fuelSubsystem;
  private final double targetRPS;

  /**
   * @param fuelSubsystem the fuel subsystem
   * @param targetRPS     target flywheel velocity in rotations per second
   *                      (negative = launch direction on this robot)
   */
  public SpinUpFlywheel(CANFuelSubsystem fuelSubsystem, double targetRPS) {
    this.fuelSubsystem = fuelSubsystem;
    this.targetRPS = targetRPS;
    addRequirements(fuelSubsystem);
  }

  /** Convenience constructor using the default launch RPS from Constants. */
  public SpinUpFlywheel(CANFuelSubsystem fuelSubsystem) {
    this(fuelSubsystem, DEFAULT_LAUNCH_RPS);
  }

  @Override
  public void initialize() {
    // Command the flywheel to the target velocity (closed-loop)
    fuelSubsystem.setFlywheelVelocity(targetRPS);
    // Run the feeder slowly to hold the ball back while spinning up
    fuelSubsystem.setFeederRoller(FLYWHEEL_SPINUP_FEEDER_PERCENT);
  }

  @Override
  public void execute() {
    // The TalonFX closed-loop runs on the motor controller itself,
    // so there is nothing to update here every loop.
  }

  @Override
  public void end(boolean interrupted) {
    // If interrupted (button released early), stop everything.
    // If finishing normally (at speed), leave the flywheel running —
    // the next command in the sequence (Launch) will take over.
    if (interrupted) {
      fuelSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    // Done when the flywheel is at the target velocity
    return fuelSubsystem.isAtTargetVelocity();
  }
}
