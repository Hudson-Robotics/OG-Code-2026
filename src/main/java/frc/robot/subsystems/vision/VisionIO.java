package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean hasTarget = false;
    public double tx = 0.0;
    public double ty = 0.0;
    public boolean poseValid = false;
    public Pose2d robotPose = Pose2d.kZero;
    public double poseTimestamp = 0.0;
    public int tagCount = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
