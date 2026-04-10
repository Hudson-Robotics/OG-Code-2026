// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANFuelSubsystem;

/**
 * Flywheel-based launch sequence that replaces the old time-based
 * LaunchSequence.
 *
 * 1. SpinUpFlywheel — closed-loop velocity ramp-up, finishes when at target
 *    RPS (with a safety timeout so we don't wait forever).
 * 2. Launch — feeds the ball into the spinning rollers.
 */
public class FlywheelLaunchSequence extends SequentialCommandGroup {

  /**
   * @param fuelSubsystem the fuel subsystem
   * @param targetRPS     desired flywheel velocity in RPS (negative = launch
   *                      direction)
   */
  public FlywheelLaunchSequence(CANFuelSubsystem fuelSubsystem, double targetRPS) {
    addCommands(
        // Spin up with closed-loop control; safety timeout of 3 s in case PID
        // can't reach the setpoint (bad gains, low battery, etc.)
        new SpinUpFlywheel(fuelSubsystem, targetRPS).withTimeout(3.0),
        // Once at speed (or timed out), feed the ball
        new Launch(fuelSubsystem));
  }

  /** Convenience constructor using the default launch RPS from Constants. */
  public FlywheelLaunchSequence(CANFuelSubsystem fuelSubsystem) {
    this(fuelSubsystem, FuelConstants.DEFAULT_LAUNCH_RPS);
  }
}
