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
import static frc.robot.Constants.FuelConstants.FLYWHEEL_KS;
import static frc.robot.Constants.FuelConstants.FLYWHEEL_KV;
import static frc.robot.Constants.FuelConstants.FLYWHEEL_KP;
import static frc.robot.Constants.FuelConstants.FLYWHEEL_VELOCITY_TOLERANCE_RPS;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANFuelSubsystem extends SubsystemBase {
  private final TalonFX leftIntakeLauncher;
  private final TalonFX rightIntakeLauncher;
  private final TalonFX conveyor;

  // Reusable closed-loop velocity request for flywheel control
  private final VelocityVoltage flywheelRequest = new VelocityVoltage(0).withSlot(0);

  // Track the current flywheel target so periodic() can show the error
  private double flywheelTargetRPS = 0.0;

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
    // Slot 0 holds the flywheel closed-loop gains (kS, kV, kP).
    var launcherConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true))
        .withSlot0(new Slot0Configs()
            .withKS(FLYWHEEL_KS)
            .withKV(FLYWHEEL_KV)
            .withKP(FLYWHEEL_KP));
    rightIntakeLauncher.getConfigurator().apply(launcherConfig);

    launcherConfig.withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive))
        .withSlot0(new Slot0Configs()
            .withKS(FLYWHEEL_KS)
            .withKV(FLYWHEEL_KV)
            .withKP(FLYWHEEL_KP));
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

    // Flywheel tuning values on the dashboard
    SmartDashboard.putNumber("Flywheel/kS", FLYWHEEL_KS);
    SmartDashboard.putNumber("Flywheel/kV", FLYWHEEL_KV);
    SmartDashboard.putNumber("Flywheel/kP", FLYWHEEL_KP);
    SmartDashboard.putNumber("Flywheel/Target RPS", 0);
    SmartDashboard.putNumber("Flywheel/Actual RPS", 0);
    SmartDashboard.putNumber("Flywheel/Error RPS", 0);
    SmartDashboard.putBoolean("Flywheel/At Target", false);
    SmartDashboard.putNumber("Flywheel/Distance (ft)", 0);
  }

  // ---------- Open-loop methods (used by Intake, Eject, etc.) ----------

  // A method to set the percent output of the intake/launcher rollers
  public void setIntakeLauncherRoller(double power) {
    leftIntakeLauncher.set(power);
    rightIntakeLauncher.set(power); // positive for shooting
  }

  // A method to set the percent output of the feeder/conveyor roller
  public void setFeederRoller(double power) {
    conveyor.set(power);
  }

  // A method to stop all rollers
  public void stop() {
    conveyor.set(0);
    leftIntakeLauncher.set(0);
    rightIntakeLauncher.set(0);
    flywheelTargetRPS = 0.0;
  }

  // ---------- Closed-loop flywheel methods ----------

  /**
   * Command the intake/launcher rollers to a target velocity using the TalonFX
   * built-in closed-loop (VelocityVoltage with Slot 0 gains: kS, kV, kP).
   * 
   * @param rps target velocity in rotations per second (negative = launch
   *            direction on this robot)
   */
  public void setFlywheelVelocity(double rps) {
    flywheelTargetRPS = rps;
    leftIntakeLauncher.setControl(flywheelRequest.withVelocity(rps));
    rightIntakeLauncher.setControl(flywheelRequest.withVelocity(rps));
  }

  /**
   * @return the current flywheel velocity in rotations per second (average of
   *         left and right)
   */
  public double getFlywheelVelocityRPS() {
    double left = leftIntakeLauncher.getVelocity().getValueAsDouble();
    double right = rightIntakeLauncher.getVelocity().getValueAsDouble();
    return (left + right) / 2.0;
  }

  /**
   * @return true if the flywheel is within the tolerance of the current target
   */
  public boolean isAtTargetVelocity() {
    return Math.abs(getFlywheelVelocityRPS() - flywheelTargetRPS) < FLYWHEEL_VELOCITY_TOLERANCE_RPS;
  }

  /**
   * Hot-reload the flywheel PID gains from the provided values.
   * Call this from the tuning command to live-update gains without redeploying.
   */
  public void updateFlywheelGains(double kS, double kV, double kP) {
    var newSlot0 = new Slot0Configs()
        .withKS(kS)
        .withKV(kV)
        .withKP(kP);
    leftIntakeLauncher.getConfigurator().apply(newSlot0);
    rightIntakeLauncher.getConfigurator().apply(newSlot0);
  }

  @Override
  public void periodic() {
    // Publish flywheel telemetry every loop so the dashboard always shows current state
    double actualRPS = getFlywheelVelocityRPS();
    SmartDashboard.putNumber("Flywheel/Actual RPS", actualRPS);
    SmartDashboard.putNumber("Flywheel/Error RPS", flywheelTargetRPS - actualRPS);
    SmartDashboard.putBoolean("Flywheel/At Target", isAtTargetVelocity());
  }
}
