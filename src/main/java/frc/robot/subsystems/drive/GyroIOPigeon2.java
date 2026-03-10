package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** IO implementation for Pigeon 2 gyro. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(PIGEON2_ID);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    // Configure the Pigeon2 mount pose — mounted with Y pointing up
    var pigeonConfig = new Pigeon2Configuration()
        .withMountPose(new MountPoseConfigs()
            .withMountPoseYaw(0)
            .withMountPosePitch(90)
            .withMountPoseRoll(0));
    pigeon.getConfigurator().apply(pigeonConfig);
    pigeon.getConfigurator().setYaw(0.0);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, yaw, yawVelocity);
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    // Negate yaw so counter-clockwise is positive (WPILib convention)
    inputs.yawPosition = Rotation2d.fromDegrees(-yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-yawVelocity.getValueAsDouble());
  }

  @Override
  public void reset() {
    pigeon.reset();
  }
}
