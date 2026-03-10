package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** DriveIO implementation for TalonFX motors (Kraken X60). */
public class DriveIOTalonFX implements DriveIO {
  private final TalonFX leftLeader = new TalonFX(LEFT_LEADER_ID);
  private final TalonFX leftFollower = new TalonFX(LEFT_FOLLOWER_ID);
  private final TalonFX rightLeader = new TalonFX(RIGHT_LEADER_ID);
  private final TalonFX rightFollower = new TalonFX(RIGHT_FOLLOWER_ID);

  private final StatusSignal<Angle> leftPosition = leftLeader.getPosition();
  private final StatusSignal<AngularVelocity> leftVelocity = leftLeader.getVelocity();
  private final StatusSignal<Voltage> leftAppliedVolts = leftLeader.getMotorVoltage();
  private final StatusSignal<Current> leftLeaderCurrent = leftLeader.getStatorCurrent();
  private final StatusSignal<Current> leftFollowerCurrent = leftFollower.getStatorCurrent();

  private final StatusSignal<Angle> rightPosition = rightLeader.getPosition();
  private final StatusSignal<AngularVelocity> rightVelocity = rightLeader.getVelocity();
  private final StatusSignal<Voltage> rightAppliedVolts = rightLeader.getMotorVoltage();
  private final StatusSignal<Current> rightLeaderCurrent = rightLeader.getStatorCurrent();
  private final StatusSignal<Current> rightFollowerCurrent = rightFollower.getStatorCurrent();

  public DriveIOTalonFX() {
    // Configure right side motors (Clockwise_Positive)
    var config = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true));

    config.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

    rightLeader.getConfigurator().apply(config);
    rightFollower.getConfigurator().apply(config);

    // Configure left side motors (CounterClockwise_Positive)
    config.withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive));
    leftLeader.getConfigurator().apply(config);
    leftFollower.getConfigurator().apply(config);

    // Set followers
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), MotorAlignmentValue.Aligned));

    // Zero encoders
    leftLeader.setPosition(0);
    rightLeader.setPosition(0);

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftLeaderCurrent,
        leftFollowerCurrent,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightLeaderCurrent,
        rightFollowerCurrent);
    leftLeader.optimizeBusUtilization();
    leftFollower.optimizeBusUtilization();
    rightLeader.optimizeBusUtilization();
    rightFollower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftLeaderCurrent,
        leftFollowerCurrent,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightLeaderCurrent,
        rightFollowerCurrent);

    // Position and velocity are already in mechanism units (wheel rotations/RPS)
    // because we set SensorToMechanismRatio = DRIVE_GEAR_RATIO
    inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
    inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps =
        new double[] {leftLeaderCurrent.getValueAsDouble(), leftFollowerCurrent.getValueAsDouble()};

    inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps =
        new double[] {rightLeaderCurrent.getValueAsDouble(), rightFollowerCurrent.getValueAsDouble()};
  }

  @Override
  public void setSpeed(double leftPercent, double rightPercent) {
    leftLeader.set(leftPercent);
    rightLeader.set(rightPercent);
  }
}
