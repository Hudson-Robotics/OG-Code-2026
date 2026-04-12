// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.LauncherConstants.*;
import static frc.robot.Constants.FuelConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.Drive;
import frc.robot.commands.auto.DriveToClimb;
import frc.robot.commands.DynamicClimbDown;
import frc.robot.commands.DynamicClimbUp;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.JiggleDown;
import frc.robot.commands.JiggleUp;
import frc.robot.commands.auto.JustShoot;
import frc.robot.commands.LaunchAtSpeed;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.FlywheelLaunchSequence;
import frc.robot.commands.FlywheelTuningCommand;
import frc.robot.commands.auto.ShootAndClimb;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller, by default it is setup to use a single controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // Third controller used for flywheel tuning in the pit / on the field
  private final CommandXboxController tunerController = new CommandXboxController(
      TUNER_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Register named commands for PathPlanner autos BEFORE building any auto.
    // These names must match the named commands used in PathPlanner auto files.
    NamedCommands.registerCommand("Intake", new Intake(fuelSubsystem));
    NamedCommands.registerCommand("Shoot", new FlywheelLaunchSequence(fuelSubsystem));
    NamedCommands.registerCommand("ClimbUp", new ClimbUp(climberSubsystem));
    NamedCommands.registerCommand("ClimbDown", new ClimbDown(climberSubsystem));

    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Shoot And Climb", new ShootAndClimb(driveSubsystem, fuelSubsystem, climberSubsystem));
    autoChooser.addOption("Drive To Climb", new DriveToClimb(driveSubsystem, climberSubsystem));
    autoChooser.addOption("Just Shoot", new JustShoot(driveSubsystem, fuelSubsystem, climberSubsystem));
    autoChooser.addOption("PP Depot And Climb", AutoBuilder.buildAuto("PP Depot And Climb"));

    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel
    operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    // While the right bumper is held, spin up the flywheel to target RPS using
    // closed-loop velocity control, then feed the ball once at speed.
    operatorController.rightBumper().whileTrue(new FlywheelLaunchSequence(fuelSubsystem));
    // While B is held, aim at the target using Limelight, then spin up and launch
    //driverController.b().whileTrue(new LaunchSequenceWithAim(fuelSubsystem, visionSubsystem, driveSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    //operatorController.a().whileTrue(new Eject(fuelSubsystem));
    // Operator A button - short range flywheel shot at SHORT_RANGE_RPS
    operatorController.a().whileTrue(new FlywheelLaunchSequence(fuelSubsystem, SHORT_RANGE_RPS));
    // Operator X button - medium range flywheel shot at MEDIUM_RANGE_RPS
    operatorController.x().whileTrue(new FlywheelLaunchSequence(fuelSubsystem, MEDIUM_RANGE_RPS));
    // Operator Y button - far range flywheel shot at FAR_RANGE_RPS
    operatorController.y().whileTrue(new FlywheelLaunchSequence(fuelSubsystem, FAR_RANGE_RPS));
    // Operator B button - far range shot (open-loop legacy)
    operatorController.b().whileTrue(new LaunchAtSpeed(fuelSubsystem, FAR_LAUNCH_SPEED, FAR_FEEDER_SPEED));
   // While the down arrow on the directional pad is held it will unclimb the robot
    driverController.povDown().whileTrue(new ClimbDown(climberSubsystem));
    // While the up arrow on the directional pad is held it will cimb the robot
    driverController.povUp().whileTrue(new ClimbUp(climberSubsystem));

    // While the left trigger on driver controller is held, climb down with variable speed
    driverController.leftTrigger(0.1).whileTrue(new DynamicClimbDown(climberSubsystem, () -> driverController.getLeftTriggerAxis()));
    // While the right trigger on driver controller is held, climb up with variable speed
    driverController.rightTrigger(0.1).whileTrue(new DynamicClimbUp(climberSubsystem, () -> driverController.getRightTriggerAxis()));

    // While the left trigger is held, jiggle the feeder roller down (negative)
    operatorController.leftTrigger(0.1).whileTrue(new JiggleDown(fuelSubsystem, () -> operatorController.getLeftTriggerAxis()));
    // While the right trigger is held, jiggle the feeder roller up (positive)
    operatorController.rightTrigger(0.1).whileTrue(new JiggleUp(fuelSubsystem, () -> operatorController.getRightTriggerAxis()));

    // ---- Tuner controller (3rd controller) bindings ----
    // Hold A to run the flywheel tuning command. While held, the flywheel spins
    // at the RPS set on the dashboard, and gains can be hot-tuned live.
    // Set "Flywheel/Feed Now" to true on the dashboard (or press B) to feed a ball.
    tunerController.a().whileTrue(new FlywheelTuningCommand(fuelSubsystem));

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));

    climberSubsystem.setDefaultCommand(climberSubsystem.run(() -> climberSubsystem.stop()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
