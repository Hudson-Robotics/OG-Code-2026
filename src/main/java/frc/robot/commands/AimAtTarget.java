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
 * Rotates the robot in place until the Limelight target is centered within
 * AIM_TOLERANCE_DEGREES. Ends immediately if no target is visible.
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

    // Proportional rotation: positive TX means target is to the right,
    // so we rotate right (positive zRotation in arcade drive)
    double rotation = MathUtil.clamp(
        visionSubsystem.getTX() * AIM_KP,
        -AIM_MAX_OUTPUT,
        AIM_MAX_OUTPUT);

    driveSubsystem.driveArcade(0, rotation);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    // End when aimed or when no target is visible
    return visionSubsystem.isAimed() || !visionSubsystem.hasTarget();
  }
}
