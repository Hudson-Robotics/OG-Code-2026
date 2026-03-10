package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import static frc.robot.Constants.VisionConstants.*;

/** VisionIO implementation using Limelight. */
public class VisionIOLimelight implements VisionIO {

  // Discard pose estimates where any visible tag has ambiguity above this threshold.
  private static final double MAX_AMBIGUITY = 0.2;

  // Discard pose estimates based on only 1 tag if it's too far away (inches)
  private static final double MAX_SINGLE_TAG_DISTANCE = 60.0;

  public VisionIOLimelight() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.hasTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);
    inputs.tx = LimelightHelpers.getTX(LIMELIGHT_NAME);
    inputs.ty = LimelightHelpers.getTY(LIMELIGHT_NAME);

    // Get pose estimate with reliability filtering
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

    if (estimate != null && estimate.tagCount > 0) {
      boolean reliable = true;

      // Reject if any tag has high ambiguity
      for (RawFiducial fiducial : estimate.rawFiducials) {
        if (fiducial.ambiguity > MAX_AMBIGUITY) {
          reliable = false;
          break;
        }
      }

      // Reject single-tag estimates that are too far away
      if (reliable && estimate.tagCount == 1 && estimate.avgTagDist > MAX_SINGLE_TAG_DISTANCE) {
        reliable = false;
      }

      if (reliable) {
        inputs.poseValid = true;
        inputs.robotPose = estimate.pose;
        inputs.poseTimestamp = estimate.timestampSeconds;
        inputs.tagCount = estimate.tagCount;
      } else {
        inputs.poseValid = false;
        inputs.tagCount = 0;
      }
    } else {
      inputs.poseValid = false;
      inputs.tagCount = 0;
    }
  }
}
