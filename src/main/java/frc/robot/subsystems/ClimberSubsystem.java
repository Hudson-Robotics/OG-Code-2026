package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstatns.*;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotor;
  private final SlewRateLimiter slewRateLimiter;

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
    climberMotor.set(slewRateLimiter.calculate(power));
  }

  // A method to stop the climber and reset the slew rate limiter
  public void stop() {
    slewRateLimiter.reset(0);
    climberMotor.set(0);
  }

  public double getPosition()
  {
    return climberMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
