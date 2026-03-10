// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

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

  // AdvantageKit mode — change simMode to Mode.REPLAY to replay from a log file
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_LEADER_ID = 2;
    public static final int RIGHT_FOLLOWER_ID = 4;

    // CAN ID for the Pigeon2 gyro
    public static final int PIGEON2_ID = 11;

    // Current limit for drivetrain motors (Kraken X60).
    // 60A stator limit is a good balance of performance and breaker safety.
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    // Wheel and drivetrain geometry — measured on the KitBot
    // Wheel diameter in meters (4-inch wheels = 0.1016 m)
    public static final double WHEEL_DIAMETER_METERS = 0.1016;
    // Distance between the left and right wheels, measured center-to-center (meters)
    public static final double TRACK_WIDTH_METERS = 0.546;
    // Gear reduction between the motor and the wheel axle (motor turns : 1 wheel turn)
    public static final double DRIVE_GEAR_RATIO = 5.143;

    // Max theoretical free speed in m/s (Kraken X60 free speed ≈ 6000 RPM)
    public static final double MAX_SPEED_MPS =
        (6000.0 / 60.0 / DRIVE_GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER_METERS);
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
    public static final double CLIMBER_MOTOR_DOWN_PERCENT = -0.90;
    public static final double CLIMBER_MOTOR_UP_PERCENT = 0.90;
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
}
