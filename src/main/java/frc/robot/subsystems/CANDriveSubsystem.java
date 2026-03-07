// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final TalonFX leftLeader;
  private final TalonFX leftFollower;
  private final TalonFX rightLeader;
  private final TalonFX rightFollower;

  private final CANcoder canCoderLeft;
  private final CANcoder canCoderRight;
  private final Pigeon2 pigeon2;

  private final DifferentialDrive drive;
  private final DifferentialDriveOdometry odometry;

  // Converts CANcoder rotations (at the wheel axle) to meters
  private static final double METERS_PER_ROTATION = Math.PI * WHEEL_DIAMETER_METERS;

  public CANDriveSubsystem() {
    // Create drive motors
    leftLeader = new TalonFX(LEFT_LEADER_ID);
    leftFollower = new TalonFX(LEFT_FOLLOWER_ID);
    rightLeader = new TalonFX(RIGHT_LEADER_ID);
    rightFollower = new TalonFX(RIGHT_FOLLOWER_ID);

    // Create sensors
    canCoderLeft = new CANcoder(CANCODER_LEFT_ID);
    canCoderRight = new CANcoder(CANCODER_RIGHT_ID);
    pigeon2 = new Pigeon2(PIGEON2_ID);

    // Set up differential drive class
    drive = new DifferentialDrive(leftLeader::set, rightLeader::set);

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

    // Zero the CANcoders and gyro, then initialize odometry at the origin
    pigeon2.reset();
    canCoderLeft.setPosition(0);
    canCoderRight.setPosition(0);
    odometry = new DifferentialDriveOdometry(
        getHeading(),
        getLeftDistanceMeters(),
        getRightDistanceMeters());
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
   * Returns the robot's current heading as a Rotation2d.
   * Yaw is negated so that counter-clockwise is positive (WPILib convention).
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-pigeon2.getYaw().getValueAsDouble());
  }

  /**
   * Returns the distance the left wheel has traveled in meters,
   * derived from the left CANcoder's accumulated position (rotations at the axle).
   */
  public double getLeftDistanceMeters() {
    return canCoderLeft.getPosition().getValueAsDouble() * METERS_PER_ROTATION;
  }

  /**
   * Returns the distance the right wheel has traveled in meters,
   * derived from the right CANcoder's accumulated position (rotations at the axle).
   */
  public double getRightDistanceMeters() {
    return canCoderRight.getPosition().getValueAsDouble() * METERS_PER_ROTATION;
  }

  /**
   * Returns the robot's estimated pose (X, Y, heading) on the field.
   * Resets to origin at robot startup.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the given pose, also zeroing the CANcoder and gyro.
   * Call this at the start of auto with the robot's known starting position.
   */
  public void resetOdometry(Pose2d pose) {
    pigeon2.reset();
    canCoderLeft.setPosition(0);
    canCoderRight.setPosition(0);
    odometry.resetPosition(getHeading(), 0, 0, pose);
  }
}
