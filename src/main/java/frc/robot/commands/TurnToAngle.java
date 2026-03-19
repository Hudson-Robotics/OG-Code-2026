package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import static frc.robot.Constants.AutoConstants.*;

/**
 * Spins the robot in place by the specified number of degrees relative to
 * the heading at the moment the command starts.
 *
 * <p>Uses a P-controller on the Pigeon2 heading. Ends when the heading
 * error is within {@code TURN_TOLERANCE_DEGREES}.
 *
 * <p>Positive degrees = clockwise (right), negative = counter-clockwise (left).
 */
public class TurnToAngle extends Command {

  private final CANDriveSubsystem driveSubsystem;
  private final double relativeDegrees;

  private double targetHeadingDeg;

  /**
   * @param driveSubsystem  the drive subsystem
   * @param relativeDegrees how many degrees to turn (positive = CW / right)
   */
  public TurnToAngle(CANDriveSubsystem driveSubsystem, double relativeDegrees) {
    this.driveSubsystem = driveSubsystem;
    this.relativeDegrees = relativeDegrees;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    double currentHeading = driveSubsystem.getHeading().getDegrees();
    targetHeadingDeg = currentHeading + relativeDegrees;
  }

  @Override
  public void execute() {
    double currentHeading = driveSubsystem.getHeading().getDegrees();
    double error = targetHeadingDeg - currentHeading;

    // Wrap error to [-180, 180] so we always take the shortest path
    while (error > 180) error -= 360;
    while (error < -180) error += 360;

    double rotation = MathUtil.clamp(error * TURN_KP, -TURN_MAX_OUTPUT, TURN_MAX_OUTPUT);
    driveSubsystem.driveArcade(0, rotation);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    double currentHeading = driveSubsystem.getHeading().getDegrees();
    double error = targetHeadingDeg - currentHeading;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    return Math.abs(error) <= TURN_TOLERANCE_DEGREES;
  }
}
