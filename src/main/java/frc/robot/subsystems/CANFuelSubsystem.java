// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.INDEXER_INTAKING_PERCENT;
import static frc.robot.Constants.FuelConstants.INDEXER_LAUNCHING_PERCENT;
import static frc.robot.Constants.FuelConstants.CONVEYOR_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.CONVEYOR_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.INTAKE_INTAKING_PERCENT;
import static frc.robot.Constants.FuelConstants.LAUNCHER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.LAUNCHING_LAUNCHER_PERCENT;
import static frc.robot.Constants.FuelConstants.LEFT_INTAKE_LAUNCHER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.RIGHT_INTAKE_LAUNCHER_MOTOR_ID;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANFuelSubsystem extends SubsystemBase {
  private final TalonFX leftIntakeLauncher;
  private final TalonFX rightIntakeLauncher;
  private final TalonFX conveyor;

  /** Creates a new CANFuelSubsystem. */
  public CANFuelSubsystem() {
    leftIntakeLauncher = new TalonFX(LEFT_INTAKE_LAUNCHER_MOTOR_ID);
    rightIntakeLauncher = new TalonFX(RIGHT_INTAKE_LAUNCHER_MOTOR_ID);
    conveyor = new TalonFX(CONVEYOR_MOTOR_ID);

    // Configure the conveyor with a current limit and brake mode
    var conveyorConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CONVEYOR_MOTOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true));
    conveyor.getConfigurator().apply(conveyorConfig);



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
    conveyor.set(power);
  }

  // A method to stop the rollers
  public void stop() {
    conveyor.set(0);
    leftIntakeLauncher.set(0);
    rightIntakeLauncher.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
