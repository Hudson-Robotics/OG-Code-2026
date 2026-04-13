// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import static frc.robot.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

  // Discard pose estimates where any visible tag has ambiguity above this threshold.
  // 0.0 = perfect, 1.0 = completely ambiguous. 0.2 is a common safe value.
  private static final double MAX_AMBIGUITY = 0.2;

  // Discard pose estimates based on only 1 tag if it's too far away (inches)
  private static final double MAX_SINGLE_TAG_DISTANCE = 60.0;

  /** WPILib's official 2026 AndyMark field layout with all AprilTag poses. */
  private final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public VisionSubsystem() {
  }

  // -------------------------------------------------------------------------
  //  Front Limelight — used for aiming and shooting at the hub
  // -------------------------------------------------------------------------

  /** Returns true if the front Limelight has a valid target. */
  public boolean hasTarget() {
    return LimelightHelpers.getTV(LIMELIGHT_FRONT_NAME);
  }

  /**
   * Returns the horizontal offset from the crosshair to the target in degrees.
   * Negative = target is to the left, Positive = target is to the right.
   */
  public double getTX() {
    return LimelightHelpers.getTX(LIMELIGHT_FRONT_NAME);
  }

  /**
   * Returns the vertical offset from the crosshair to the target in degrees.
   * Negative = target is below, Positive = target is above.
   */
  public double getTY() {
    return LimelightHelpers.getTY(LIMELIGHT_FRONT_NAME);
  }

  /** Returns true if the robot is aimed at the target within the tolerance. */
  public boolean isAimed() {
    return hasTarget() && Math.abs(getTX()) < AIM_TOLERANCE_DEGREES;
  }

  /**
   * Returns the raw PoseEstimate from the front Limelight using the WPILib Blue
   * origin, or null if no tags are visible or the estimate is considered unreliable.
   *
   * Reliability checks:
   *  - At least one tag must be visible
   *  - All visible tags must have ambiguity below MAX_AMBIGUITY
   *  - If only one tag is visible, it must be within MAX_SINGLE_TAG_DISTANCE inches
   */
  public PoseEstimate getPoseEstimate() {
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_FRONT_NAME);

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
   * Returns the robot's estimated field-relative Pose2d from the front Limelight,
   * or null if the estimate is unavailable or unreliable.
   */
  public Pose2d getRobotPose() {
    PoseEstimate estimate = getPoseEstimate();
    return estimate != null ? estimate.pose : null;
  }

  /**
   * Returns a list of AprilTag objects (from the WPILib field layout) for all
   * tags currently visible on the front Limelight.
   */
  public List<AprilTag> getVisibleTags() {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(LIMELIGHT_FRONT_NAME);
    List<AprilTag> tags = new ArrayList<>();
    if (fiducials == null || fiducials.length == 0) {
      return tags;
    }
    for (RawFiducial f : fiducials) {
      Optional<Pose3d> pose = fieldLayout.getTagPose(f.id);
      if (pose.isPresent()) {
        tags.add(new AprilTag(f.id, pose.get()));
      }
    }
    return tags;
  }

  /**
   * Returns the average distance (in inches) from the robot to all currently
   * visible AprilTags on the front Limelight, or -1 if no tags are visible.
   */
  public double getAvgTagDistance() {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(LIMELIGHT_FRONT_NAME);
    if (fiducials == null || fiducials.length == 0) {
      return -1;
    }
    double sum = 0;
    for (RawFiducial f : fiducials) {
      sum += f.distToRobot;
    }
    return sum / fiducials.length;
  }

  /**
   * Returns true if the robot is both aimed (TX within tolerance) and at
   * the target distance from the hub (within distanceToleranceInches).
   */
  public boolean isAimedAndInRange(double targetDistanceInches, double distanceToleranceInches) {
    if (!isAimed()) return false;
    double dist = getAvgTagDistance();
    if (dist < 0) return false;
    return Math.abs(dist - targetDistanceInches) <= distanceToleranceInches;
  }

  // -------------------------------------------------------------------------
  //  Hub distance & alignment helpers
  // -------------------------------------------------------------------------

  /**
   * Returns the hub center point for the current alliance as a Translation2d
   * (in meters). Uses the average of the hub AprilTag positions from the WPILib
   * field layout. Defaults to Blue alliance if no alliance info is available.
   */
  public Translation2d getHubCenter() {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    return isRed ? RED_HUB_CENTER : BLUE_HUB_CENTER;
  }

  /**
   * Returns the distance in inches from the robot's current Limelight pose
   * estimate to the hub center for the current alliance.
   * Returns -1 if no reliable pose estimate is available.
   */
  public double getDistanceToHubInches() {
    Pose2d pose = getRobotPose();
    if (pose == null) {
      return -1;
    }
    Translation2d hub = getHubCenter();
    double distMeters = pose.getTranslation().getDistance(hub);
    return distMeters / 0.0254; // convert meters to inches
  }

  /**
   * Returns the WPILib field layout used by this subsystem.
   */
  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }

  // -------------------------------------------------------------------------
  //  Back Limelight — available for future use (intake tracking, etc.)
  // -------------------------------------------------------------------------

  /** Returns true if the back Limelight has a valid target. */
  public boolean hasTargetBack() {
    return LimelightHelpers.getTV(LIMELIGHT_BACK_NAME);
  }

  /** Returns the horizontal offset from the back Limelight crosshair in degrees. */
  public double getTXBack() {
    return LimelightHelpers.getTX(LIMELIGHT_BACK_NAME);
  }

  /** Returns the vertical offset from the back Limelight crosshair in degrees. */
  public double getTYBack() {
    return LimelightHelpers.getTY(LIMELIGHT_BACK_NAME);
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

    List<AprilTag> tags = getVisibleTags();
    SmartDashboard.putNumber("Vision/Visible Tag Count", tags.size());
    for (int i = 0; i < tags.size(); i++) {
      SmartDashboard.putString("Vision/Tag " + i, tags.get(i).toString());
    }

    // Hub distance telemetry
    double hubDist = getDistanceToHubInches();
    SmartDashboard.putNumber("Vision/Hub Distance (in)", hubDist);

    // Back Limelight telemetry
    SmartDashboard.putBoolean("Vision/Back Has Target", hasTargetBack());
    SmartDashboard.putNumber("Vision/Back TX", getTXBack());
    SmartDashboard.putNumber("Vision/Back TY", getTYBack());
  }
}
