// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity. 
 */
public final class Constants {

  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_LEADER_ID = 2;
    public static final int RIGHT_FOLLOWER_ID = 4;

    // CAN IDs for drivetrain sensors
    public static final int CANCODER_LEFT_ID = 9;
    public static final int CANCODER_RIGHT_ID = 10;
    public static final int PIGEON2_ID = 11;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    // Wheel and drivetrain geometry — measure these on your robot and update!
    // Wheel diameter in meters (6-inch Colson = ~0.1524 m)
    public static final double WHEEL_DIAMETER_METERS = 0.1524;
    // Distance between the left and right wheels, measured center-to-center (meters)
    public static final double TRACK_WIDTH_METERS = 0.56;
    // Gear reduction between the motor and the wheel axle (motor turns : 1 wheel turn)
    public static final double DRIVE_GEAR_RATIO = 10.71;

    // Maximum rate of change for drive output (units per second).
    // A value of 1.0 means it takes 1 second to go from 0 to full speed.
    // Lower values = smoother but slower response. Higher = snappier.
    public static final double DRIVE_SLEW_RATE = 4;
    public static final double ROTATION_SLEW_RATE = 2.0;

    // Clamp drive output to this maximum magnitude (0 to 1)
    public static final double MAX_DRIVE_OUTPUT = 1.0;
    public static final double MAX_ROTATION_OUTPUT = 1.0;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int LEFT_INTAKE_LAUNCHER_MOTOR_ID = 5;
    public static final int RIGHT_INTAKE_LAUNCHER_MOTOR_ID = 6;
    public static final int INDEXER_MOTOR_ID = 8;

    // Current limit for fuel mechanism motors.
    public static final int INDEXER_MOTOR_CURRENT_LIMIT = 80;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 100;

    // All values likely need to be tuned based on your robot
    public static final double INDEXER_INTAKING_PERCENT = -.8; 
    public static final double INDEXER_LAUNCHING_PERCENT = 0.6;
    public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.5;

    public static final double INTAKE_INTAKING_PERCENT = -0.7;
    public static final double LAUNCHING_LAUNCHER_PERCENT = -.95;
    public static final double INTAKE_EJECT_PERCENT = 0.9;

    public static final double SPIN_UP_SECONDS = 1;
  }

  public static final class ClimbConstatns {
    // Motor controller IDs for Climb motor
    public static final int CLIMBER_MOTOR_ID = 7;

    // Current limit for climb motor
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 80;
    // Percentage to power the motor both up and down
    public static final double CLIMBER_MOTOR_DOWN_PERCENT = -0.95;
    public static final double CLIMBER_MOTOR_UP_PERCENT = 0.95;

    public static final double CLIMBER_SLEW_RATE = 1.0;
  }

  public static final class OperatorConstants {

    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = 0.7;
    public static final double ROTATION_SCALING = 0.8;
  }

  public static final class VisionConstants {
    // Name of the Limelight as configured in the Limelight web interface
    public static final String LIMELIGHT_NAME = "limelight";

    // How close to centered (in degrees TX) we need to be before considered aimed
    public static final double AIM_TOLERANCE_DEGREES = 1.5;

    // Proportional gain for the aiming rotation output. Tune this on the robot:
    // increase if turning is too slow, decrease if it oscillates
    public static final double AIM_KP = 0.035;

    // Maximum rotation speed (0-1) while aiming to avoid overshooting
    public static final double AIM_MAX_OUTPUT = 0.5;
  }

  public static final class AutoConstants {
    // DriveToClimb auto
    public static final double DRIVE_TO_CLIMB_WAIT_SECONDS = 3.0;
    public static final double DRIVE_TO_CLIMB_DRIVE_SECONDS = 2.0;
    public static final double DRIVE_TO_CLIMB_SPEED = 0.5;

    // JustShoot auto
    public static final double JUST_SHOOT_DRIVE_SPEED = 0.4;
    public static final double JUST_SHOOT_DRIVE_SECONDS = 1.5;
    public static final double JUST_SHOOT_SECONDS = 5.0;

    // ShootAndClimb auto
    public static final double SHOOT_AND_CLIMB_DRIVE_SPEED = 0.5;
    public static final double SHOOT_AND_CLIMB_DRIVE_SECONDS = 2.0;
    public static final double SHOOT_AND_CLIMB_SHOOT_SECONDS = 4.0;
  }

  public static final class LauncherConstants {
    // Launcher roller speeds for different distances — tune these on the field
    public static final double SHORT_LAUNCH_SPEED = -0.4;
    public static final double MEDIUM_LAUNCH_SPEED = -0.65;
    public static final double FAR_LAUNCH_SPEED = -0.95;

    // Jiggler (feeder) speed when launching at different distances
    public static final double SHORT_FEEDER_SPEED = 0.4;
    public static final double MEDIUM_FEEDER_SPEED = 0.5;
    public static final double FAR_FEEDER_SPEED = 0.6;
  }
}
