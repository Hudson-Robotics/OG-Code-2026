package frc.robot.subsystems.fuel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Fuel (intake/launcher/feeder) subsystem using AdvantageKit IO layer.
 */
public class FuelSubsystem extends SubsystemBase {
  private final FuelIO io;
  private final FuelIOInputsAutoLogged inputs = new FuelIOInputsAutoLogged();

  public FuelSubsystem(FuelIO io) {
    this.io = io;

    // Put default tuning values on the dashboard
    SmartDashboard.putNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
    SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Fuel", inputs);
  }

  /** Sets the intake/launcher roller speed. */
  public void setIntakeLauncherRoller(double power) {
    io.setIntakeLauncherSpeed(power);
  }

  /** Sets the feeder/jiggler roller speed. */
  public void setFeederRoller(double power) {
    io.setFeederSpeed(power);
  }

  /** Stops all fuel motors. */
  public void stop() {
    io.stop();
  }
}
