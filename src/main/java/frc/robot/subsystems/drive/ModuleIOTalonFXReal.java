package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

public class ModuleIOTalonFXReal extends ModuleIOTalonFX {

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  public ModuleIOTalonFXReal(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {

    super(constants);

    this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    this.drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(super.drivePositionSignal.clone());
    this.turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(super.turnPositionSignal.clone());
  }

  @Override
  public void readInputs(ModuleInputs inputs) {
    super.readInputs(inputs);

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();

    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map((Rotation2d::fromRotations)).toArray(Rotation2d[]::new);

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }
}
