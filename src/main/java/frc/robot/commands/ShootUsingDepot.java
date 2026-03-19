package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.AutoConstants.*;

/**
 * Autonomous routine: Shoot Using Depot.
 *
 * <ol>
 *   <li>Drive straight (Pigeon-corrected) with intake running for N seconds
 *       to pick up fuel from the depot.</li>
 *   <li>Back up for N/2 seconds.</li>
 *   <li>Spin in place to 135° (roughly diagonal to the hub).</li>
 *   <li>Fine-aim at the hub using the front Limelight
 *       (AprilTags 8 &amp; 9 for Red, 24 &amp; 25 for Blue).</li>
 *   <li>Spin up the launcher and shoot.</li>
 * </ol>
 */
public class ShootUsingDepot extends SequentialCommandGroup {

  public ShootUsingDepot(CANDriveSubsystem driveSubsystem,
                         CANFuelSubsystem fuelSubsystem,
                         VisionSubsystem visionSubsystem) {
    addCommands(
        // 1. Drive straight with intake on for N seconds
        new StraightDriveWithIntake(driveSubsystem, fuelSubsystem, DEPOT_DRIVE_SPEED)
            .withTimeout(DEPOT_DRIVE_SECONDS),

        // 2. Back up for N/2 seconds (Pigeon-corrected straight line)
        new StraightDrive(driveSubsystem, DEPOT_BACKUP_SPEED)
            .withTimeout(DEPOT_BACKUP_SECONDS),

        // 3. Spin in place to 135° to face the hub diagonally
        new TurnToAngle(driveSubsystem, DEPOT_SPIN_DEGREES)
            .withTimeout(3.0),

        // 4. Fine-aim using the front Limelight (tags 8/9 red, 24/25 blue)
        new AimAtTarget(visionSubsystem, driveSubsystem)
            .withTimeout(2.0),

        // 5. Spin up the launcher, then shoot
        new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        new Launch(fuelSubsystem).withTimeout(DEPOT_SHOOT_SECONDS));
  }
}
