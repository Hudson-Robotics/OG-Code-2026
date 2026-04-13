// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

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

    // Clamp drive output to this maximum magnitude (0 to 1)
    public static final double MAX_DRIVE_OUTPUT = 1.0;
    public static final double MAX_ROTATION_OUTPUT = 1.0;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int LEFT_INTAKE_LAUNCHER_MOTOR_ID = 5;
    public static final int RIGHT_INTAKE_LAUNCHER_MOTOR_ID = 6;
    public static final int CONVEYOR_MOTOR_ID = 29;

    // Current limit for fuel mechanism motors.
    public static final int CONVEYOR_MOTOR_CURRENT_LIMIT = 80;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 100;

    // All values likely need to be tuned based on your robot
    // ---- Ball path reference ----
    // Negative intake + positive feeder = field → intake → hopper (INTAKING)
    // Negative intake + negative feeder = hopper → shooter (SHOOTING)
    // Positive intake + positive feeder = hopper → back out intake (EJECT)
    // Positive intake + negative feeder = intake → shooter / bypass hopper (SHUTTLE/PASS)
    //
    // Intake from field into hopper (-intake, +feeder)
    public static final double INTAKE_ROLLER_PERCENT = -0.7;
    public static final double INTAKE_FEEDER_PERCENT = 0.8;
    // Feed from hopper to shooter (-feeder pushes toward shooter)
    public static final double LAUNCH_FEEDER_PERCENT = -0.7;
    // Eject back out of the intake (+intake, +feeder)
    public static final double EJECT_ROLLER_PERCENT = 0.9;
    public static final double EJECT_FEEDER_PERCENT = 0.8;
    // Shuttle/pass: bypass hopper, go straight from intake to shooter (+intake, -feeder)
    public static final double SHUTTLE_ROLLER_PERCENT = 0.7;
    public static final double SHUTTLE_FEEDER_PERCENT = -0.7;

    // Legacy / unused
    public static final double INDEXER_INTAKING_PERCENT = -.8; 
    public static final double INDEXER_LAUNCHING_PERCENT = -0.7;
    public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.6;
    public static final double INTAKE_INTAKING_PERCENT = -0.7;
    public static final double LAUNCHING_LAUNCHER_PERCENT = -.95;
    public static final double INTAKE_EJECT_PERCENT = 0.9;

    public static final double SPIN_UP_SECONDS = 1;

    // ---- Flywheel closed-loop constants (tune these!) ----
    // kS: voltage to overcome static friction (volts). Start small.
    public static final double FLYWHEEL_KS = 0.15;
    // kV: voltage per unit of velocity (volts / RPS). Main feedforward term.
    public static final double FLYWHEEL_KV = 0.12;
    // kP: proportional gain on velocity error (volts / RPS error)
    public static final double FLYWHEEL_KP = 0.1;

    // How close the flywheel must be to the target before we consider it ready (RPS)
    public static final double FLYWHEEL_VELOCITY_TOLERANCE_RPS = 3.0;
    // Default target flywheel velocity for launching (rotations per second)
    // Negative because your launch direction is negative. Tune on the robot.
    public static final double DEFAULT_LAUNCH_RPS = -80.0;
    // Preset flywheel velocities for different shooting distances (RPS)
    public static final double SHORT_RANGE_RPS = -70.0;
    public static final double MEDIUM_RANGE_RPS = -80.0;
    public static final double FAR_RANGE_RPS = -90.0;
    // Shuttle/pass flywheel velocity (RPS)
    public static final double SHUTTLE_RPS = -90.0;
    // Feeder speed while the flywheel is spinning up (slow reverse to hold the ball back)
    public static final double FLYWHEEL_SPINUP_FEEDER_PERCENT = -0.05;
  }

  public static final class ClimbConstatns {
    // Motor controller IDs for Climb motor
    public static final int CLIMBER_MOTOR_ID = 7;

    // Current limit for climb motor
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 80;
    // Percentage to power the motor both up and down
    public static final double CLIMBER_MOTOR_DOWN_PERCENT = -0.3867;
    public static final double CLIMBER_MOTOR_UP_PERCENT = 0.95;
  }

  public static final class OperatorConstants {

    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    // Third controller used for flywheel tuning in the pit / on the field
    public static final int TUNER_CONTROLLER_PORT = 2;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = 0.7;
    public static final double ROTATION_SCALING = 0.8;
  }

  public static final class VisionConstants {
    // Names of the Limelights as configured in the Limelight web interface
    public static final String LIMELIGHT_NAME = "limelight";
    public static final String LIMELIGHT_FRONT_NAME = "limelight";
    public static final String LIMELIGHT_BACK_NAME = "limelight-back";

    // How close to centered (in degrees TX) we need to be before considered aimed
    public static final double AIM_TOLERANCE_DEGREES = 1.5;

    // Proportional gain for the aiming rotation output. Tune this on the robot:
    // increase if turning is too slow, decrease if it oscillates
    public static final double AIM_KP = 0.035;

    // Maximum rotation speed (0-1) while aiming to avoid overshooting
    public static final double AIM_MAX_OUTPUT = 0.5;

    // Minimum rotation output — below this the motors stall and draw current
    // without moving. Any output smaller than this is snapped to zero.
    public static final double AIM_DEADBAND = 0.05;

    // Target distance from the hub in inches (~6 feet)
    public static final double AIM_TARGET_DISTANCE_INCHES = 72.0;
    // How close to the target distance (in inches) before we consider it "in range"
    public static final double AIM_DISTANCE_TOLERANCE_INCHES = 4.0;
    // Proportional gain for the distance (forward/back) correction
    public static final double AIM_DISTANCE_KP = 0.006;
    // Maximum forward/backward speed while ranging to avoid overshooting
    public static final double AIM_DISTANCE_MAX_OUTPUT = 0.35;

    // Proportional gain for the Pigeon heading correction during straight driving
    public static final double HEADING_CORRECTION_KP = 0.02;

    // ---- Hub center points (meters, WPILib Blue-origin coordinate system) ----
    // Computed from the average of all hub AprilTag positions in the 2026 AndyMark layout.
    // Red hub tags: 2,3,4,5,8,9,10,11   Blue hub tags: 18,19,20,21,24,25,26,27
    public static final Translation2d RED_HUB_CENTER = new Translation2d(11.9903, 4.0214);
    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.5227, 4.0214);

    // ---- Distance-to-RPS interpolation for auto-aim shooting ----
    // Maps distance (inches) → flywheel RPS (negative = launch direction).
    // Linear interpolation is used between points; values outside the range are
    // clamped to the nearest endpoint.
    // Add more calibration points here as you tune on the field.
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_RPS_MAP = new InterpolatingDoubleTreeMap();
    static {
      // (distance in inches, flywheel RPS — negative for launch direction)
      DISTANCE_TO_RPS_MAP.put(30.0, -60.0);
      DISTANCE_TO_RPS_MAP.put(120.0, -90.0);
    }
  }

  public static final class AutoConstants {
    // DriveToClimb auto
    public static final double DRIVE_TO_CLIMB_WAIT_SECONDS = 3.0;
    public static final double DRIVE_TO_CLIMB_DRIVE_SECONDS = 2.0;
    public static final double DRIVE_TO_CLIMB_SPEED = 0.5;

    // JustShoot auto
    public static final double JUST_SHOOT_DRIVE_SPEED = 0.4;
    public static final double JUST_SHOOT_DRIVE_SECONDS = 1.867;
    public static final double JUST_SHOOT_SECONDS = 15.0;

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
