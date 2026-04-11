// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Flywheel tuning command designed for use with a dedicated 3rd controller in
 * the pit or on the field.
 *
 * <h2>Dashboard workflow</h2>
 * <ol>
 *   <li>Set <b>Flywheel/kS</b>, <b>Flywheel/kV</b>, <b>Flywheel/kP</b> to starting values.</li>
 *   <li>Set <b>Flywheel/Target RPS</b> to the velocity you want to test.</li>
 *   <li>Hold a button to run this command — the flywheel spins up closed-loop.</li>
 *   <li>Watch <b>Flywheel/Actual RPS</b> and <b>Flywheel/At Target</b>.</li>
 *   <li>When At Target is true, set <b>Flywheel/Feed Now</b> to true on the
 *       dashboard (or press the feed button on the tuner controller) to feed one
 *       ball.</li>
 *   <li>Measure where the ball lands, record the distance and RPS.</li>
 *   <li>Adjust gains or target and repeat.</li>
 * </ol>
 *
 * The command reads gains from the dashboard every loop so you can hot-tune
 * without redeploying.
 *
 * <h2>Tuning Procedure — start ALL gains at 0, tune in order: kS → kV → kP</h2>
 *
 * <h3>Step 1: kS (static friction compensation)</h3>
 * <ul>
 *   <li>kS is a constant voltage applied whenever the motor is asked to move.
 *       It overcomes friction so the motor can start spinning.</li>
 *   <li>Set Target RPS to something small (e.g. -5). kV and kP stay at 0.</li>
 *   <li>Increase kS by +0.05 at a time.</li>
 *   <li><b>DONE when:</b> the motor barely starts to creep / you see Actual RPS
 *       go from 0 to a very small number. Typical value: 0.1 – 0.3 V.</li>
 *   <li>Too low → motor doesn't move. Too high → motor spins fast on its own.</li>
 * </ul>
 *
 * <h3>Step 2: kV (velocity feedforward — the "F" in the PF loop)</h3>
 * <ul>
 *   <li>kV is volts-per-RPS. It predicts how much voltage the motor needs for
 *       a given speed. This is the main power source.</li>
 *   <li>Keep kS at its tuned value, kP stays at 0.</li>
 *   <li>Set Target RPS to your real target (e.g. -60 to -80).</li>
 *   <li>Start kV at 0.05 and increase by +0.01 to +0.02 at a time.</li>
 *   <li><b>DONE when:</b> Actual RPS reaches ~90-95% of Target RPS and holds
 *       steady. Error RPS will be small but not zero — that's fine.</li>
 *   <li>Sanity check: try different Target RPS values (-40, -60, -80).
 *       The motor should roughly track all of them proportionally.</li>
 *   <li>Too low → Actual way below Target. Too high → Actual overshoots Target.</li>
 *   <li>Typical value: 0.10 – 0.15 V/RPS.</li>
 * </ul>
 *
 * <h3>Step 3: kP (proportional error correction)</h3>
 * <ul>
 *   <li>kP closes the remaining gap that kS + kV can't cover. It adds voltage
 *       proportional to (Target - Actual).</li>
 *   <li>Keep kS and kV at their tuned values.</li>
 *   <li>Start kP at 0.01 and increase by +0.01 to +0.05 at a time.</li>
 *   <li><b>DONE when:</b> "Flywheel/At Target" goes TRUE, Error RPS stays
 *       within ±3 of zero, and the speed recovers quickly after feeding a ball.</li>
 *   <li>Too low → Error RPS still large. Too high → Actual RPS oscillates
 *       (bounces above and below target, motor pulses/buzzes). Back it down.</li>
 *   <li>Typical value: 0.05 – 0.3 V/RPS-error.</li>
 * </ul>
 *
 * <h3>After tuning</h3>
 * <p>Copy the final kS, kV, kP values from the dashboard into Constants.java
 * (FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KP). Then use this command to test
 * different Target RPS values and record distance-to-RPS pairs for your
 * distance lookup table.</p>
 */
public class FlywheelTuningCommand extends Command {

  private final CANFuelSubsystem fuelSubsystem;

  // Track the last gains we applied so we only re-apply on change
  private double lastKS, lastKV, lastKP;

  public FlywheelTuningCommand(CANFuelSubsystem fuelSubsystem) {
    this.fuelSubsystem = fuelSubsystem;
    addRequirements(fuelSubsystem);
  }

  @Override
  public void initialize() {
    // Seed "Feed Now" to false so the operator has to deliberately enable it
    //SmartDashboard.putBoolean("Flywheel/Feed Now", false);

    // Read and apply the current gains from the dashboard
    lastKS = SmartDashboard.getNumber("Flywheel/kS", FLYWHEEL_KS);
    lastKV = SmartDashboard.getNumber("Flywheel/kV", FLYWHEEL_KV);
    lastKP = SmartDashboard.getNumber("Flywheel/kP", FLYWHEEL_KP);
    fuelSubsystem.updateFlywheelGains(lastKS, lastKV, lastKP);

    // Start spinning at whatever Target RPS is on the dashboard
    double targetRPS = SmartDashboard.getNumber("Flywheel/Target RPS", DEFAULT_LAUNCH_RPS);
    fuelSubsystem.setFlywheelVelocity(targetRPS);

    // Hold feeder still while spinning up
    fuelSubsystem.setFeederRoller(0);
  }

  @Override
  public void execute() {
    // ---- Live-update target RPS ----
    double targetRPS = SmartDashboard.getNumber("Flywheel/Target RPS", DEFAULT_LAUNCH_RPS);
    SmartDashboard.putNumber("Flywheel/Target RPS", targetRPS);
    fuelSubsystem.setFlywheelVelocity(targetRPS);

    // ---- Live-update gains if they changed on the dashboard ----
    double kS = SmartDashboard.getNumber("Flywheel/kS", FLYWHEEL_KS);
    double kV = SmartDashboard.getNumber("Flywheel/kV", FLYWHEEL_KV);
    double kP = SmartDashboard.getNumber("Flywheel/kP", FLYWHEEL_KP);

    if (kS != lastKS || kV != lastKV || kP != lastKP) {
      fuelSubsystem.updateFlywheelGains(kS, kV, kP);
      lastKS = kS;
      lastKV = kV;
      lastKP = kP;
    }

    // ---- Feed on demand ----
    // When the operator sets "Flywheel/Feed Now" to true, run the feeder to
    // launch one ball. The operator should toggle it back to false after firing.
    //boolean feedNow = SmartDashboard.getBoolean("Flywheel/Feed Now", false);
    if (fuelSubsystem.isAtTargetVelocity()) {
      fuelSubsystem.setFeederRoller(INDEXER_LAUNCHING_PERCENT);
    } else {
      // Keep feeder stopped (or at a gentle hold-back) while not feeding
      fuelSubsystem.setFeederRoller(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
    //SmartDashboard.putBoolean("Flywheel/Feed Now", false);
  }

  @Override
  public boolean isFinished() {
    // Runs until the button is released (whileTrue binding)
    return false;
  }
}
