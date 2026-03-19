package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstatns.*;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotor;
  private final SlewRateLimiter slewRateLimiter;

  // PID state for position control
  private double targetPosition = Double.NaN;
  private double previousError = 0;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberMotor = new TalonFX(CLIMBER_MOTOR_ID);
    slewRateLimiter = new SlewRateLimiter(CLIMBER_SLEW_RATE);

    // Configure the climber motor with a current limit and brake mode
    var climbConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true));

    climberMotor.getConfigurator().apply(climbConfig);
  }

  // A method to set the percentage of the climber, ramped by the slew rate limiter
  public void setClimber(double power) {
    clearTargetPosition();
    climberMotor.set(slewRateLimiter.calculate(power));
  }

  // A method to stop the climber and reset the slew rate limiter
  public void stop() {
    clearTargetPosition();
    slewRateLimiter.reset(0);
    climberMotor.set(0);
  }

  public double getPosition() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  // ---- Position control (PID) ----

  /**
   * Sets a target position for the climber. The periodic() loop will drive
   * the motor toward this position using a PID controller with asymmetric
   * output limits (slower going down to protect the 80:1 gears).
   */
  public void setTargetPosition(double position) {
    this.targetPosition = position;
    this.previousError = 0;
  }

  /** Clears the position target so periodic() stops driving the motor. */
  public void clearTargetPosition() {
    this.targetPosition = Double.NaN;
    this.previousError = 0;
  }

  /** Returns true if a position target is active. */
  public boolean hasTargetPosition() {
    return !Double.isNaN(targetPosition);
  }

  /** Returns true if the climber is within tolerance of the target position. */
  public boolean atTargetPosition() {
    if (!hasTargetPosition()) return false;
    return Math.abs(getPosition() - targetPosition) <= CLIMB_POSITION_TOLERANCE;
  }

  @Override
  public void periodic() {
    double currentPos = getPosition();
    SmartDashboard.putNumber("Climb/Position", currentPos);

    // If we have an active position target, run the PID loop
    if (hasTargetPosition()) {
      double error = targetPosition - currentPos;
      double derivative = error - previousError;
      previousError = error;

      double output = (CLIMB_KP * error) + (CLIMB_KI * error) + (CLIMB_KD * derivative);

      // Asymmetric output clamping: allow more power going up, limit going down
      // Negative output = motor going down (toward more negative positions)
      // Positive output = motor going up (toward 0 / floor)
      if (output < 0) {
        // Going down — limit speed to protect gears
        output = MathUtil.clamp(output, -CLIMB_MAX_DOWN_OUTPUT, 0);
      } else {
        // Going up — allow more power against gravity
        output = MathUtil.clamp(output, 0, CLIMB_MAX_UP_OUTPUT);
      }

      climberMotor.set(output);

      SmartDashboard.putNumber("Climb/Target", targetPosition);
      SmartDashboard.putNumber("Climb/Error", error);
      SmartDashboard.putNumber("Climb/Output", output);
      SmartDashboard.putBoolean("Climb/At Target", atTargetPosition());
    }
  }
}
