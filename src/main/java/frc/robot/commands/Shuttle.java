// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Shuttle / pass command — spins the flywheel to SHUTTLE_RPS and feeds the
 * ball straight through to the shooter side, bypassing the hopper.
 *
 * Uses closed-loop flywheel velocity so the pass speed is consistent.
 * The feeder runs negative (toward shooter) as soon as the flywheel is at speed.
 */
public class Shuttle extends Command {

  private final CANFuelSubsystem fuelSubsystem;

  public Shuttle(CANFuelSubsystem fuelSubsystem) {
    this.fuelSubsystem = fuelSubsystem;
    addRequirements(fuelSubsystem);
  }

  @Override
  public void initialize() {
    // Start the flywheel at shuttle speed (closed-loop)
    fuelSubsystem.setFlywheelVelocity(SHUTTLE_RPS);
    // Hold feeder while spinning up
    fuelSubsystem.setFeederRoller(0);
  }

  @Override
  public void execute() {
    // Keep commanding the flywheel every loop
    fuelSubsystem.setFlywheelVelocity(SHUTTLE_RPS);

    // Once at speed, feed the ball through toward the shooter (-feeder)
    if (fuelSubsystem.isAtTargetVelocity()) {
      fuelSubsystem.setFeederRoller(SHUTTLE_FEEDER_PERCENT);
    } else {
      fuelSubsystem.setFeederRoller(0);
    }
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
