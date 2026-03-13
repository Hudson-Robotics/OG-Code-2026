package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class DynamicClimbUp extends Command {

  private final ClimberSubsystem climberSubsystem;
  private final DoubleSupplier triggerValue;

  public DynamicClimbUp(ClimberSubsystem climberSubsystem, DoubleSupplier triggerValue) {
    this.climberSubsystem = climberSubsystem;
    this.triggerValue = triggerValue;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Use the trigger value (positive) to control climb speed
    climberSubsystem.setClimber(triggerValue.getAsDouble());
    SmartDashboard.putNumber("Pos", climberSubsystem.getPosition());
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    //return climberSubsystem.getPosition() > 60;
    return false;
  }
}
