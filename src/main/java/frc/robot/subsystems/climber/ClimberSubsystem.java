package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Climber subsystem using AdvantageKit IO layer.
 */
public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  /** Sets the climber motor speed. */
  public void setClimber(double power) {
    io.setSpeed(power);
  }

  /** Stops the climber motor. */
  public void stop() {
    io.stop();
  }

  /** Returns the climber position in radians. */
  public double getPosition() {
    return inputs.positionRad;
  }
}
