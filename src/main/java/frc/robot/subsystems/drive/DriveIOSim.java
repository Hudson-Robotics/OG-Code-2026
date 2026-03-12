package frc.robot.subsystems.drive;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * DriveIO simulation implementation using WPILib's DCMotorSim.
 * Models left and right sides of a differential drive independently.
 */
public class DriveIOSim implements DriveIO {
  // Kraken X60 is modeled as a Falcon500 (very similar motor)
  private static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(2); // 2 motors per side

  private final DCMotorSim leftSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DRIVE_MOTOR, 0.025, DRIVE_GEAR_RATIO),
      DRIVE_MOTOR);

  private final DCMotorSim rightSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DRIVE_MOTOR, 0.025, DRIVE_GEAR_RATIO),
      DRIVE_MOTOR);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // Advance simulation by 20ms (one robot loop)
    leftSim.update(0.02);
    rightSim.update(0.02);

    inputs.leftPositionRad = leftSim.getAngularPositionRad();
    inputs.leftVelocityRadPerSec = leftSim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {leftSim.getCurrentDrawAmps()};

    inputs.rightPositionRad = rightSim.getAngularPositionRad();
    inputs.rightVelocityRadPerSec = rightSim.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {rightSim.getCurrentDrawAmps()};
  }

  @Override
  public void setSpeed(double leftPercent, double rightPercent) {
    // Convert percent (-1 to 1) to voltage (assuming 12V battery)
    leftAppliedVolts = leftPercent * 12.0;
    rightAppliedVolts = rightPercent * 12.0;
    leftSim.setInputVoltage(leftAppliedVolts);
    rightSim.setInputVoltage(rightAppliedVolts);
  }
}
