// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTagLocation;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import static frc.robot.Constants.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {

  // Discard pose estimates where any visible tag has ambiguity above this threshold.
  // 0.0 = perfect, 1.0 = completely ambiguous. 0.2 is a common safe value.
  private static final double MAX_AMBIGUITY = 0.2;

  // Discard pose estimates based on only 1 tag if it's too far away (inches)
  private static final double MAX_SINGLE_TAG_DISTANCE = 60.0;

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

  /**
   * Returns the raw PoseEstimate from the Limelight using the WPILib Blue origin,
   * or null if no tags are visible or the estimate is considered unreliable.
   *
   * Reliability checks:
   *  - At least one tag must be visible
   *  - All visible tags must have ambiguity below MAX_AMBIGUITY
   *  - If only one tag is visible, it must be within MAX_SINGLE_TAG_DISTANCE inches
   */
  public PoseEstimate getPoseEstimate() {
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

    if (estimate == null || estimate.tagCount == 0) {
      return null;
    }

    // Reject if any tag has high ambiguity
    for (RawFiducial fiducial : estimate.rawFiducials) {
      if (fiducial.ambiguity > MAX_AMBIGUITY) {
        return null;
      }
    }

    // Reject single-tag estimates that are too far away to be accurate
    if (estimate.tagCount == 1 && estimate.avgTagDist > MAX_SINGLE_TAG_DISTANCE) {
      return null;
    }

    return estimate;
  }

  /**
   * Returns the robot's estimated field-relative Pose2d, or null if the
   * estimate is unavailable or unreliable.
   */
  public Pose2d getRobotPose() {
    PoseEstimate estimate = getPoseEstimate();
    return estimate != null ? estimate.pose : null;
  }

  /**
   * Returns an array of AprilTagLocations for all tags currently visible,
   * skipping any tag IDs not found in the AprilTagLocation enum.
   */
  public AprilTagLocation[] getVisibleTags() {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(LIMELIGHT_NAME);
    if (fiducials == null || fiducials.length == 0) {
      return new AprilTagLocation[0];
    }

    // Count valid tags first to size the array
    int validCount = 0;
    for (RawFiducial f : fiducials) {
      if (AprilTagLocation.fromId(f.id) != null) validCount++;
    }

    AprilTagLocation[] tags = new AprilTagLocation[validCount];
    int i = 0;
    for (RawFiducial f : fiducials) {
      AprilTagLocation tag = AprilTagLocation.fromId(f.id);
      if (tag != null) tags[i++] = tag;
    }
    return tags;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
    SmartDashboard.putNumber("Vision/TX", getTX());
    SmartDashboard.putNumber("Vision/TY", getTY());
    SmartDashboard.putBoolean("Vision/Is Aimed", isAimed());

    Pose2d pose = getRobotPose();
    SmartDashboard.putBoolean("Vision/Pose Valid", pose != null);
    if (pose != null) {
      SmartDashboard.putNumber("Vision/Pose X", pose.getX());
      SmartDashboard.putNumber("Vision/Pose Y", pose.getY());
      SmartDashboard.putNumber("Vision/Pose Heading", pose.getRotation().getDegrees());
    }

    AprilTagLocation[] tags = getVisibleTags();
    SmartDashboard.putNumber("Vision/Visible Tag Count", tags.length);
    for (int i = 0; i < tags.length; i++) {
      SmartDashboard.putString("Vision/Tag " + i, tags[i].toString());
    }
  }
}
