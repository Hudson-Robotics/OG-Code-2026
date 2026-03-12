package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;

public class JiggleDown extends Command {

  private final CANFuelSubsystem fuelSubsystem;
  private final DoubleSupplier triggerValue;

  public JiggleDown(CANFuelSubsystem fuelSubsystem, DoubleSupplier triggerValue) {
    this.fuelSubsystem = fuelSubsystem;
    this.triggerValue = triggerValue;
    addRequirements(fuelSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Pass the trigger value as negative to jiggle down
    fuelSubsystem.setFeederRoller(-triggerValue.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.setFeederRoller(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
