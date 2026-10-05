package frc.robot.subsystems.drive;

import frc.robot.Util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSim;

  public GyroIOSim(GyroSimulation gyroSim) {
    this.gyroSim = gyroSim;
  }

  @Override
  public void readInputs(GyroInputs inputs) {
    inputs.gyroConnected = true;
    inputs.gyroYaw = gyroSim.getGyroReading().getMeasure();
    inputs.gyroYawVelocity = gyroSim.getMeasuredAngularVelocity();

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = gyroSim.getCachedGyroReadings();
  }
}
