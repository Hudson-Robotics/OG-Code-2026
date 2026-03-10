package frc.robot.subsystems.climber;

import static frc.robot.Constants.ClimbConstatns.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** ClimberIO implementation for a single TalonFX motor. */
public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX climberMotor = new TalonFX(CLIMBER_MOTOR_ID);

  private final StatusSignal<Angle> position = climberMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = climberMotor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = climberMotor.getMotorVoltage();
  private final StatusSignal<Current> current = climberMotor.getStatorCurrent();

  public ClimberIOTalonFX() {
    var config = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true));
    climberMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    climberMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setSpeed(double percent) {
    climberMotor.set(percent);
  }

  @Override
  public void stop() {
    climberMotor.set(0);
  }
}
