package frc.robot.subsystems.fuel;

import static frc.robot.Constants.FuelConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** FuelIO implementation for real hardware (2× TalonFX launcher + 1× SparkMax feeder). */
public class FuelIOTalonFX implements FuelIO {
  private final TalonFX leftLauncher = new TalonFX(LEFT_INTAKE_LAUNCHER_MOTOR_ID);
  private final TalonFX rightLauncher = new TalonFX(RIGHT_INTAKE_LAUNCHER_MOTOR_ID);
  private final SparkMax feeder = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);

  private final StatusSignal<AngularVelocity> leftVelocity = leftLauncher.getVelocity();
  private final StatusSignal<Voltage> leftAppliedVolts = leftLauncher.getMotorVoltage();
  private final StatusSignal<Current> leftCurrent = leftLauncher.getStatorCurrent();

  private final StatusSignal<AngularVelocity> rightVelocity = rightLauncher.getVelocity();
  private final StatusSignal<Voltage> rightAppliedVolts = rightLauncher.getMotorVoltage();
  private final StatusSignal<Current> rightCurrent = rightLauncher.getStatorCurrent();

  public FuelIOTalonFX() {
    // Configure feeder (SparkMax brushed) current limit
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
    feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure right launcher (Clockwise_Positive, Coast mode)
    var launcherConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true));
    rightLauncher.getConfigurator().apply(launcherConfig);

    // Configure left launcher (CounterClockwise_Positive, Coast mode)
    launcherConfig.withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive));
    leftLauncher.getConfigurator().apply(launcherConfig);

    // Set status signal update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftVelocity, leftAppliedVolts, leftCurrent,
        rightVelocity, rightAppliedVolts, rightCurrent);
    leftLauncher.optimizeBusUtilization();
    rightLauncher.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FuelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftVelocity, leftAppliedVolts, leftCurrent,
        rightVelocity, rightAppliedVolts, rightCurrent);

    inputs.leftLauncherVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.leftLauncherAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftLauncherCurrentAmps = leftCurrent.getValueAsDouble();

    inputs.rightLauncherVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
    inputs.rightLauncherAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightLauncherCurrentAmps = rightCurrent.getValueAsDouble();

    inputs.feederAppliedOutput = feeder.getAppliedOutput();
    inputs.feederCurrentAmps = feeder.getOutputCurrent();
  }

  @Override
  public void setIntakeLauncherSpeed(double percent) {
    leftLauncher.set(percent);
    rightLauncher.set(percent);
  }

  @Override
  public void setFeederSpeed(double percent) {
    feeder.set(percent);
  }

  @Override
  public void stop() {
    leftLauncher.set(0);
    rightLauncher.set(0);
    feeder.set(0);
  }
}
