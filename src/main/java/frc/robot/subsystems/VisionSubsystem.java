// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import static frc.robot.Constants.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {

  public VisionSubsystem() {
  }

  /** Returns true if the Limelight has a valid target. */
  public boolean hasTarget() {
    return LimelightHelpers.getTV(LIMELIGHT_NAME);
  }

  /**
   * Returns the horizontal offset from the crosshair to the target in degrees.
   * Negative = target is to the left, Positive = target is to the right.
   */
  public double getTX() {
    return LimelightHelpers.getTX(LIMELIGHT_NAME);
  }

  /**
   * Returns the vertical offset from the crosshair to the target in degrees.
   * Negative = target is below, Positive = target is above.
   */
  public double getTY() {
    return LimelightHelpers.getTY(LIMELIGHT_NAME);
  }

  /** Returns true if the robot is aimed at the target within the tolerance. */
  public boolean isAimed() {
    return hasTarget() && Math.abs(getTX()) < AIM_TOLERANCE_DEGREES;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
    SmartDashboard.putNumber("Vision/TX", getTX());
    SmartDashboard.putNumber("Vision/TY", getTY());
    SmartDashboard.putBoolean("Vision/Is Aimed", isAimed());
  }
}
