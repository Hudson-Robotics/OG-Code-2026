// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final TalonFX leftLeader;
  private final TalonFX leftFollower;
  private final TalonFX rightLeader;
  private final TalonFX rightFollower;

  private final Pigeon2 pigeon2;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;
  private final DifferentialDriveKinematics kinematics;

  // Converts TalonFX motor rotations to meters at the wheel
  // TalonFX reports position in motor rotations; divide by gear ratio to get wheel rotations
  private static final double METERS_PER_MOTOR_ROTATION =
      (Math.PI * WHEEL_DIAMETER_METERS) / DRIVE_GEAR_RATIO;

  public CANDriveSubsystem() {
    // Create drive motors
    leftLeader = new TalonFX(LEFT_LEADER_ID);
    leftFollower = new TalonFX(LEFT_FOLLOWER_ID);
    rightLeader = new TalonFX(RIGHT_LEADER_ID);
    rightFollower = new TalonFX(RIGHT_FOLLOWER_ID);

    // Create the Pigeon2 gyro
    pigeon2 = new Pigeon2(PIGEON2_ID);

    // Configure Pigeon2 mount pose — the Pigeon is mounted with Y pointing up
    // instead of the default Z-up. This requires a pitch offset of 90°.
    var pigeonConfig = new Pigeon2Configuration()
        .withMountPose(new MountPoseConfigs()
            .withMountPoseYaw(0)
            .withMountPosePitch(90)
            .withMountPoseRoll(0));
    pigeon2.getConfigurator().apply(pigeonConfig);

    // Set up differential drive class
    drive = new DifferentialDrive(leftLeader::set, rightLeader::set);

    // Set up kinematics for converting ChassisSpeeds <-> wheel speeds
    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

    // Create the configuration to apply to motors. Brake mode keeps the robot
    // from rolling when stopped. The current limit helps prevent tripping breakers.
    var config = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true));

    // Apply config to right side motors
    rightLeader.getConfigurator().apply(config);
    rightFollower.getConfigurator().apply(config);

    // Left side is inverted so that positive values drive both sides forward
    config.withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive));
    leftLeader.getConfigurator().apply(config);
    leftFollower.getConfigurator().apply(config);

    // Set followers to follow their respective leaders.
    // Aligned means the follower uses the same invert as the leader.
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), MotorAlignmentValue.Aligned));

    // Zero the built-in motor encoders and gyro, then initialize odometry at the origin
    pigeon2.reset();
    leftLeader.setPosition(0);
    rightLeader.setPosition(0);
    odometry = new DifferentialDriveOdometry(
        getHeading(),
        getLeftDistanceMeters(),
        getRightDistanceMeters());

    // Configure PathPlanner AutoBuilder for differential drive
    configureAutoBuilder();
  }

  /**
   * Configures PathPlannerLib's AutoBuilder for this differential drivetrain.
   * Uses the PPLTVController (built-in path following controller for diff drive).
   */
  private void configureAutoBuilder() {
    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }

    AutoBuilder.configure(
        this::getPose,                // Robot pose supplier
        this::resetPose,              // Method to reset odometry
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier (MUST BE ROBOT RELATIVE)
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Drive the robot given robot-relative ChassisSpeeds
        new PPLTVController(0.02),    // Path following controller for differential drive
        robotConfig,                  // Robot configuration from PathPlanner GUI
        () -> {
          // Flip path for red alliance (origin stays on blue side)
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this                          // Subsystem requirement
    );
  }

  @Override
  public void periodic() {
    // Update odometry every loop with the latest heading and per-side distances
    odometry.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());

    // Publish sensor data to SmartDashboard for tuning and debugging
    Pose2d pose = getPose();
    SmartDashboard.putNumber("Drive/Heading Deg", getHeading().getDegrees());
    SmartDashboard.putNumber("Drive/Left Distance M", getLeftDistanceMeters());
    SmartDashboard.putNumber("Drive/Right Distance M", getRightDistanceMeters());
    SmartDashboard.putNumber("Drive/Pose X", pose.getX());
    SmartDashboard.putNumber("Drive/Pose Y", pose.getY());
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * Drives the robot using robot-relative ChassisSpeeds.
   * Used by PathPlanner to follow trajectories.
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    // Convert m/s wheel speeds to percent output (approximate: divide by max speed)
    // Max free speed = (motor free speed RPM / 60 / gear ratio) * wheel circumference
    // TalonFX (Falcon 500) free speed ≈ 6380 RPM
    double maxSpeed = (6380.0 / 60.0 / DRIVE_GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER_METERS);
    leftLeader.set(wheelSpeeds.leftMetersPerSecond / maxSpeed);
    rightLeader.set(wheelSpeeds.rightMetersPerSecond / maxSpeed);
  }

  /**
   * Returns the current robot-relative ChassisSpeeds, calculated from wheel velocities.
   * Required by PathPlanner AutoBuilder.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
        getLeftVelocityMetersPerSecond(),
        getRightVelocityMetersPerSecond());
    return kinematics.toChassisSpeeds(wheelSpeeds);
  }

  /**
   * Returns the robot's current heading as a Rotation2d.
   * Yaw is negated so that counter-clockwise is positive (WPILib convention).
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-pigeon2.getYaw().getValueAsDouble());
  }

  /**
   * Returns the distance the left wheel has traveled in meters,
   * derived from the left TalonFX's built-in encoder (motor rotations / gear ratio).
   */
  public double getLeftDistanceMeters() {
    return leftLeader.getPosition().getValueAsDouble() * METERS_PER_MOTOR_ROTATION;
  }

  /**
   * Returns the distance the right wheel has traveled in meters,
   * derived from the right TalonFX's built-in encoder (motor rotations / gear ratio).
   */
  public double getRightDistanceMeters() {
    return rightLeader.getPosition().getValueAsDouble() * METERS_PER_MOTOR_ROTATION;
  }

  /**
   * Returns the left wheel velocity in meters per second,
   * derived from the left TalonFX's built-in encoder velocity (rotations per second).
   */
  public double getLeftVelocityMetersPerSecond() {
    return leftLeader.getVelocity().getValueAsDouble() * METERS_PER_MOTOR_ROTATION;
  }

  /**
   * Returns the right wheel velocity in meters per second,
   * derived from the right TalonFX's built-in encoder velocity (rotations per second).
   */
  public double getRightVelocityMetersPerSecond() {
    return rightLeader.getVelocity().getValueAsDouble() * METERS_PER_MOTOR_ROTATION;
  }

  /**
   * Returns the robot's estimated pose (X, Y, heading) on the field.
   * Resets to origin at robot startup.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the robot's pose to the given pose.
   * Called by PathPlanner when an auto has a starting pose.
   */
  public void resetPose(Pose2d pose) {
    leftLeader.setPosition(0);
    rightLeader.setPosition(0);
    odometry.resetPosition(getHeading(), 0, 0, pose);
  }

  /**
   * Resets the odometry to the given pose, also zeroing encoders and gyro.
   * Call this at the start of auto with the robot's known starting position.
   */
  public void resetOdometry(Pose2d pose) {
    pigeon2.reset();
    resetPose(pose);
  }
}
