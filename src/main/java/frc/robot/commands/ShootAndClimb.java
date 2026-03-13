package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.AutoConstants.*;

/**
 * Autonomous routine: drive slowly while shooting at the same time,
 * then climb once shooting/driving is done.
 */
public class ShootAndClimb extends SequentialCommandGroup {

  public ShootAndClimb(CANDriveSubsystem driveSubsystem, CANFuelSubsystem fuelSubsystem,
      ClimberSubsystem climberSubsystem) {
    addCommands(
        // Drive slowly and shoot at the same time
        new ClimbUp(climberSubsystem),
        new ParallelCommandGroup(
            // Drive at a slower speed for the configured time
            new AutoDrive(driveSubsystem, SHOOT_AND_CLIMB_DRIVE_SPEED, 0.0)
                .withTimeout(SHOOT_AND_CLIMB_DRIVE_SECONDS),
            // Spin up then launch
            new SequentialCommandGroup(
                new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
                new Launch(fuelSubsystem).withTimeout(SHOOT_AND_CLIMB_SHOOT_SECONDS))),
        // Once both are done, climb
        new ClimbDown(climberSubsystem));
  }
}
