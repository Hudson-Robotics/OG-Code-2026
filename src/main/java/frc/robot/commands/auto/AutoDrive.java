// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * Drives the robot at a fixed speed and rotation for use in autonomous routines.
 * Typically used with .withTimeout() to drive for a set duration.
 */
public class AutoDrive extends Command {
  CANDriveSubsystem driveSubsystem;
  double xSpeed, zRotation;

  public AutoDrive(CANDriveSubsystem driveSystem, double xSpeed, double zRotation) {
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
  }

  @Override
  public void initialize() {
  }

  // Setting the values here instead of in initialize feeds the watchdog on the
  // arcade drive object
  @Override
  public void execute() {
    driveSubsystem.driveArcade(xSpeed, zRotation);
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
