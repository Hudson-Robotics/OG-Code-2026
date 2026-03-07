// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  private final TalonFX leftIntakeLauncher;
  private final TalonFX rightIntakeLauncher;
  private final TalonFX indexer;

  /** Creates a new CANFuelSubsystem. */
  public CANFuelSubsystem() {
    leftIntakeLauncher = new TalonFX(LEFT_INTAKE_LAUNCHER_MOTOR_ID);
    rightIntakeLauncher = new TalonFX(RIGHT_INTAKE_LAUNCHER_MOTOR_ID);
    indexer = new TalonFX(INDEXER_MOTOR_ID);

    // Configure the indexer with a current limit and brake mode
    var indexerConfig = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true));
    indexer.getConfigurator().apply(indexerConfig);

    // Configure the launcher rollers with a current limit and coast mode.
    // Right is not inverted; left is inverted so positive values intake and launch
    // on both sides.
    var launcherConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true));
    rightIntakeLauncher.getConfigurator().apply(launcherConfig);

    launcherConfig.withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive));
    leftIntakeLauncher.getConfigurator().apply(launcherConfig);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
    SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT);
    //SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double power) {
    leftIntakeLauncher.set(power);
    rightIntakeLauncher.set(power); // positive for shooting
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double power) {
    indexer.set(power); // positive for shooting
  }

  // A method to stop the rollers
  public void stop() {
    indexer.set(0);
    leftIntakeLauncher.set(0);
    rightIntakeLauncher.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
