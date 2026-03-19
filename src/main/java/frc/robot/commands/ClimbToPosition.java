package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * Smoothly moves the climber hook to a preset encoder position using the
 * PID loop running in ClimberSubsystem's periodic().
 *
 * <p>Asymmetric speed limits protect the 80:1 gears: going down is slower
 * than going up. The command ends once the position is within tolerance.
 */
public class ClimbToPosition extends Command {

  private final ClimberSubsystem climberSubsystem;
  private final double targetPosition;

  /**
   * @param climberSubsystem the climber subsystem
   * @param targetPosition   target in motor rotations (e.g. 0 = floor, −230 = tier 1, −460 = tier 2)
   */
  public ClimbToPosition(ClimberSubsystem climberSubsystem, double targetPosition) {
    this.climberSubsystem = climberSubsystem;
    this.targetPosition = targetPosition;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    climberSubsystem.setTargetPosition(targetPosition);
  }

  @Override
  public void execute() {
    // PID runs in ClimberSubsystem.periodic()
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return climberSubsystem.atTargetPosition();
  }
}
