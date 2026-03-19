package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.AutoConstants.*;

/**
 * Autonomous routine: Shoot Using Neutral Zone Left.
 *
 * <ol>
 *   <li>Drive straight (Pigeon-corrected) with intake running for N seconds.</li>
 *   <li>Back up for 1.2N seconds.</li>
 *   <li>Spin in place −45° (counter-clockwise) to face the hub.</li>
 *   <li>Fine-aim at the hub using the front Limelight
 *       (AprilTags 8 &amp; 9 for Red, 24 &amp; 25 for Blue — same as depot,
 *       since this is on the left-hand side).</li>
 *   <li>Spin up the launcher and shoot.</li>
 * </ol>
 */
public class ShootUsingNeutralZoneLeft extends SequentialCommandGroup {

  public ShootUsingNeutralZoneLeft(CANDriveSubsystem driveSubsystem,
                                    CANFuelSubsystem fuelSubsystem,
                                    VisionSubsystem visionSubsystem) {
    addCommands(
        // 1. Drive straight with intake on for N seconds
        new StraightDriveWithIntake(driveSubsystem, fuelSubsystem, NZ_LEFT_DRIVE_SPEED)
            .withTimeout(NZ_LEFT_DRIVE_SECONDS),

        // 2. Back up for 1.2N seconds
        new StraightDrive(driveSubsystem, NZ_LEFT_BACKUP_SPEED)
            .withTimeout(NZ_LEFT_BACKUP_SECONDS),

        // 3. Spin −45° (counter-clockwise / left) to face the hub
        new TurnToAngle(driveSubsystem, NZ_LEFT_SPIN_DEGREES)
            .withTimeout(3.0),

        // 4. Fine-aim using the front Limelight (tags 8/9 red, 24/25 blue)
        new AimAtTarget(visionSubsystem, driveSubsystem)
            .withTimeout(2.0),

        // 5. Spin up the launcher, then shoot
        new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        new Launch(fuelSubsystem).withTimeout(NZ_LEFT_SHOOT_SECONDS));
  }
}
