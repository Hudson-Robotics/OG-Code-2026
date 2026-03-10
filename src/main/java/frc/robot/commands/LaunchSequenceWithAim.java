// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.fuel.FuelSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class LaunchSequenceWithAim extends SequentialCommandGroup {
  /** Creates a new LaunchSequenceWithAim. Aims at the target first, then spins up and launches. */
  public LaunchSequenceWithAim(FuelSubsystem fuelSubsystem, VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    addCommands(
        new AimAtTarget(visionSubsystem, driveSubsystem),
        new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        new Launch(fuelSubsystem));
  }
}
