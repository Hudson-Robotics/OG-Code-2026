package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.*;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Drive subsystem using AdvantageKit IO layers.
 * Accepts DriveIO and GyroIO for hardware abstraction and replay support.
 */
public class DriveSubsystem extends SubsystemBase {
  private final DriveIO driveIO;
  private final GyroIO gyroIO;
  private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;

  // Wheel radius in meters (used to convert from radians to meters)
  private static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2.0;

  // Track the raw heading derived from wheel encoders (used when no gyro is connected)
  private double rawHeadingRad = 0.0;

  public DriveSubsystem(DriveIO driveIO, GyroIO gyroIO) {
    this.driveIO = driveIO;
    this.gyroIO = gyroIO;

    kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    odometry = new DifferentialDriveOdometry(
        Rotation2d.kZero, 0.0, 0.0);

    configureAutoBuilder();
  }

  /**
   * Configures PathPlannerLib's AutoBuilder for this differential drivetrain.
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
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPLTVController(0.02),
        robotConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  @Override
  public void periodic() {
    // Update IO inputs and log them
    driveIO.updateInputs(driveInputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive", driveInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    // Update odometry — use gyro heading if connected, otherwise fall back to
    // integrating wheel speeds through kinematics
    double leftDistanceMeters = driveInputs.leftPositionRad * WHEEL_RADIUS_METERS;
    double rightDistanceMeters = driveInputs.rightPositionRad * WHEEL_RADIUS_METERS;

    if (gyroInputs.connected) {
      odometry.update(gyroInputs.yawPosition, leftDistanceMeters, rightDistanceMeters);
    } else {
      // No gyro — derive heading from the difference in wheel distances.
      // (right - left) / trackWidth gives the heading in radians.
      rawHeadingRad = (rightDistanceMeters - leftDistanceMeters) / TRACK_WIDTH_METERS;
      odometry.update(new Rotation2d(rawHeadingRad), leftDistanceMeters, rightDistanceMeters);
    }

    // Log the pose — use Pose2d[] so AdvantageScope can display it on the field widget
    Pose2d pose = getPose();
    Logger.recordOutput("Odometry/Robot", pose);
    Logger.recordOutput("Odometry/X", pose.getX());
    Logger.recordOutput("Odometry/Y", pose.getY());
    Logger.recordOutput("Odometry/HeadingDeg", pose.getRotation().getDegrees());
  }

  /** Drives the robot using arcade controls. */
  public void driveArcade(double xSpeed, double zRotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    driveIO.setSpeed(speeds.left, speeds.right);
  }

  /** Drives the robot using robot-relative ChassisSpeeds (used by PathPlanner). */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    driveIO.setSpeed(
        wheelSpeeds.leftMetersPerSecond / MAX_SPEED_MPS,
        wheelSpeeds.rightMetersPerSecond / MAX_SPEED_MPS);
  }

  /** Returns the current robot-relative ChassisSpeeds. */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    double leftVelocityMps = driveInputs.leftVelocityRadPerSec * WHEEL_RADIUS_METERS;
    double rightVelocityMps = driveInputs.rightVelocityRadPerSec * WHEEL_RADIUS_METERS;
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(leftVelocityMps, rightVelocityMps));
  }

  /** Returns the robot's current heading. */
  public Rotation2d getHeading() {
    return gyroInputs.connected
        ? gyroInputs.yawPosition
        : getPose().getRotation();
  }

  /** Returns the robot's estimated field pose. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Resets the robot's pose to the given pose. */
  public void resetPose(Pose2d pose) {
    double leftDistanceMeters = driveInputs.leftPositionRad * WHEEL_RADIUS_METERS;
    double rightDistanceMeters = driveInputs.rightPositionRad * WHEEL_RADIUS_METERS;
    odometry.resetPosition(getHeading(), leftDistanceMeters, rightDistanceMeters, pose);
  }

  /** Resets odometry including the gyro. */
  public void resetOdometry(Pose2d pose) {
    gyroIO.reset();
    resetPose(pose);
  }
}
