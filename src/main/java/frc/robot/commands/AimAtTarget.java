// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import static frc.robot.Constants.VisionConstants.*;

/**
 * Rotates the robot in place to center the Limelight target AND drives
 * forward/backward until the robot is approximately 6 feet (~72 inches)
 * from the hub AprilTags. Ends when both the horizontal aim and distance
 * are within tolerance, or immediately if no target is visible.
 */
public class AimAtTarget extends Command {

  private final VisionSubsystem visionSubsystem;
  private final CANDriveSubsystem driveSubsystem;

  public AimAtTarget(VisionSubsystem visionSubsystem, CANDriveSubsystem driveSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    addRequirements(visionSubsystem, driveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (!visionSubsystem.hasTarget()) {
      driveSubsystem.driveArcade(0, 0);
      return;
    }

    // --- Rotation: centre the target horizontally ---
    // Positive TX means target is to the right → rotate right (positive zRotation)
    double rotation = MathUtil.clamp(
        visionSubsystem.getTX() * AIM_KP,
        -AIM_MAX_OUTPUT,
        AIM_MAX_OUTPUT);

    // --- Distance: drive forward/backward to reach target distance ---
    double speed = 0;
    double currentDistance = visionSubsystem.getAvgTagDistance();
    if (currentDistance > 0) {
      // Positive error = too far away → drive forward (positive speed)
      // Negative error = too close   → drive backward (negative speed)
      double distanceError = currentDistance - AIM_TARGET_DISTANCE_INCHES;
      speed = MathUtil.clamp(
          distanceError * AIM_DISTANCE_KP,
          -AIM_DISTANCE_MAX_OUTPUT,
          AIM_DISTANCE_MAX_OUTPUT);
    }

    driveSubsystem.driveArcade(speed, rotation);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    if (!visionSubsystem.hasTarget()) {
      return true;
    }
    // End when both aimed AND at the correct distance
    return visionSubsystem.isAimedAndInRange(
        AIM_TARGET_DISTANCE_INCHES, AIM_DISTANCE_TOLERANCE_INCHES);
  }
}
