package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import static frc.robot.Constants.VisionConstants.*;

/**
 * Vision subsystem using AdvantageKit IO layer.
 */
public class VisionSubsystem extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public VisionSubsystem(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
  }

  /** Returns true if the Limelight has a valid target. */
  public boolean hasTarget() {
    return inputs.hasTarget;
  }

  /** Returns the horizontal offset from the crosshair to the target in degrees. */
  public double getTX() {
    return inputs.tx;
  }

  /** Returns the vertical offset from the crosshair to the target in degrees. */
  public double getTY() {
    return inputs.ty;
  }

  /** Returns true if the robot is aimed at the target within the tolerance. */
  public boolean isAimed() {
    return inputs.hasTarget && Math.abs(inputs.tx) < AIM_TOLERANCE_DEGREES;
  }

  /** Returns true if the vision pose estimate is valid. */
  public boolean isPoseValid() {
    return inputs.poseValid;
  }

  /** Returns the robot's estimated field-relative Pose2d, or null if unavailable. */
  public Pose2d getRobotPose() {
    return inputs.poseValid ? inputs.robotPose : null;
  }

  /** Returns the timestamp of the last valid pose estimate. */
  public double getPoseTimestamp() {
    return inputs.poseTimestamp;
  }

  /** Returns the number of visible tags. */
  public int getTagCount() {
    return inputs.tagCount;
  }
}
