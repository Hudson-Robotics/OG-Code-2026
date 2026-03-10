package frc.robot.subsystems.fuel;

import org.littletonrobotics.junction.AutoLog;

public interface FuelIO {
  @AutoLog
  public static class FuelIOInputs {
    public double leftLauncherVelocityRadPerSec = 0.0;
    public double leftLauncherAppliedVolts = 0.0;
    public double leftLauncherCurrentAmps = 0.0;

    public double rightLauncherVelocityRadPerSec = 0.0;
    public double rightLauncherAppliedVolts = 0.0;
    public double rightLauncherCurrentAmps = 0.0;

    public double feederAppliedOutput = 0.0;
    public double feederCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FuelIOInputs inputs) {}

  /** Sets the intake/launcher roller speed (-1 to 1). */
  public default void setIntakeLauncherSpeed(double percent) {}

  /** Sets the feeder/jiggler roller speed (-1 to 1). */
  public default void setFeederSpeed(double percent) {}

  /** Stops all motors. */
  public default void stop() {}
}
