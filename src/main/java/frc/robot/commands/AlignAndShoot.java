// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Aligns the robot to the center of the hub (rotates in place using the
 * Limelight TX offset) while simultaneously spinning the flywheel to a
 * distance-based RPS. Once aimed and at speed, the feeder fires the ball.
 *
 * The RPS is determined by linearly interpolating the robot's distance to
 * the hub center using the calibration map in Constants (DISTANCE_TO_RPS_MAP).
 *
 * Runs until the button is released (whileTrue binding).
 */
public class AlignAndShoot extends Command {

  private final VisionSubsystem visionSubsystem;
  private final CANDriveSubsystem driveSubsystem;
  private final CANFuelSubsystem fuelSubsystem;

  private double targetRPS = 0;

  public AlignAndShoot(
      VisionSubsystem visionSubsystem,
      CANDriveSubsystem driveSubsystem,
      CANFuelSubsystem fuelSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.fuelSubsystem = fuelSubsystem;
    addRequirements(visionSubsystem, driveSubsystem, fuelSubsystem);
  }

  @Override
  public void initialize() {
    targetRPS = 0;
  }

  @Override
  public void execute() {
    // --- If no target visible, stop everything ---
    if (!visionSubsystem.hasTarget()) {
      driveSubsystem.driveArcade(0, 0);
      fuelSubsystem.setFeederRoller(FLYWHEEL_SPINUP_FEEDER_PERCENT);
      return;
    }

    // --- Rotation: align to the hub center, NOT just the visible tag ---
    // getAngleToHubDegrees() computes the heading error from the robot's
    // pose to the actual hub center point for the current alliance.
    double headingError = visionSubsystem.getAngleToHubDegrees();
    double rotation = MathUtil.clamp(
        headingError * AIM_KP,
        -AIM_MAX_OUTPUT,
        AIM_MAX_OUTPUT);
    // Deadband: snap small outputs to zero so we don't stall motors
    if (Math.abs(rotation) < AIM_DEADBAND) {
      rotation = 0;
    }
    driveSubsystem.driveArcade(0, rotation);

    // --- Distance-based RPS lookup ---
    double distanceInches = visionSubsystem.getDistanceToHubInches();
    if (distanceInches > 0) {
      targetRPS = DISTANCE_TO_RPS_MAP.get(distanceInches);
    }
    // If we couldn't get a distance, keep the last known targetRPS

    // Publish telemetry so drivers/pit crew can see what's happening
    SmartDashboard.putNumber("AlignAndShoot/Distance (in)", distanceInches);
    SmartDashboard.putNumber("AlignAndShoot/Target RPS", targetRPS);
    SmartDashboard.putNumber("AlignAndShoot/Heading Error", headingError);
    SmartDashboard.putBoolean("AlignAndShoot/Aimed At Hub", visionSubsystem.isAimedAtHub());
    SmartDashboard.putBoolean("AlignAndShoot/At Speed", fuelSubsystem.isAtTargetVelocity());

    // --- Flywheel: spin up to the computed RPS ---
    if (targetRPS != 0) {
      fuelSubsystem.setFlywheelVelocity(targetRPS);
    }

    // --- Feeder: fire only when aimed at hub center AND flywheel is at speed ---
    if (visionSubsystem.isAimedAtHub() && fuelSubsystem.isAtTargetVelocity() && targetRPS != 0) {
      fuelSubsystem.setFeederRoller(LAUNCH_FEEDER_PERCENT);
    } else {
      fuelSubsystem.setFeederRoller(FLYWHEEL_SPINUP_FEEDER_PERCENT);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
    fuelSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    // Runs until the button is released
    return false;
  }
}
