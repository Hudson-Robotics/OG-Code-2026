package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import static frc.robot.Constants.VisionConstants.*;

/**
 * Drives straight at the given speed while using the Pigeon2 gyro to
 * correct heading drift, keeping the robot on a straight line.
 *
 * <p>This command never finishes on its own — use {@code .withTimeout()} to
 * limit how long it runs.
 */
public class StraightDrive extends Command {

  private final CANDriveSubsystem driveSubsystem;
  private final double speed;

  private double targetHeadingDeg;

  /**
   * @param driveSubsystem the drive subsystem
   * @param speed          forward speed (-1 to 1, positive = forward)
   */
  public StraightDrive(CANDriveSubsystem driveSubsystem, double speed) {
    this.driveSubsystem = driveSubsystem;
    this.speed = speed;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    // Lock the target heading to wherever the robot is pointing right now
    targetHeadingDeg = driveSubsystem.getHeading().getDegrees();
  }

  @Override
  public void execute() {
    // P-controller on heading error to keep the robot driving straight
    double currentHeading = driveSubsystem.getHeading().getDegrees();
    double error = targetHeadingDeg - currentHeading;
    double correction = MathUtil.clamp(error * HEADING_CORRECTION_KP, -0.3, 0.3);

    driveSubsystem.driveArcade(speed, correction);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
