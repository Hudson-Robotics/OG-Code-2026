package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Drives straight at the given speed while using the Pigeon2 gyro to
 * correct heading drift. Simultaneously runs the intake rollers so the
 * robot picks up fuel while driving.
 *
 * <p>This command never finishes on its own — use {@code .withTimeout()} to
 * limit how long it runs.
 */
public class StraightDriveWithIntake extends Command {

  private final CANDriveSubsystem driveSubsystem;
  private final CANFuelSubsystem fuelSubsystem;
  private final double speed;

  private double targetHeadingDeg;

  /**
   * @param driveSubsystem the drive subsystem
   * @param fuelSubsystem  the fuel/intake subsystem
   * @param speed          forward speed (-1 to 1, positive = forward)
   */
  public StraightDriveWithIntake(CANDriveSubsystem driveSubsystem,
                                  CANFuelSubsystem fuelSubsystem,
                                  double speed) {
    this.driveSubsystem = driveSubsystem;
    this.fuelSubsystem = fuelSubsystem;
    this.speed = speed;
    addRequirements(driveSubsystem, fuelSubsystem);
  }

  @Override
  public void initialize() {
    // Lock the target heading to wherever the robot is pointing right now
    targetHeadingDeg = driveSubsystem.getHeading().getDegrees();

    // Start the intake rollers
    fuelSubsystem.setIntakeLauncherRoller(INTAKE_INTAKING_PERCENT);
    fuelSubsystem.setFeederRoller(INDEXER_INTAKING_PERCENT);
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
    fuelSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
