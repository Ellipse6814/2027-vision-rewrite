package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.RobotConstants;
import java.util.Queue;

public class GyroIOPigeon2 implements GyroIO {

  private final Pigeon2 m_Gyro;

  private final StatusSignal<Angle> yawSignal;
  private final StatusSignal<Angle> pitchSignal;
  private final StatusSignal<Angle> rollSignal;
  private final StatusSignal<AngularVelocity> yawVelocitySignal;
  private final StatusSignal<AngularVelocity> pitchVelocitySignal;
  private final StatusSignal<AngularVelocity> rollVelocitySignal;
  private final StatusSignal<LinearAcceleration> robotXAcelerationSignal;
  private final StatusSignal<LinearAcceleration> robotYAcelerationSignal;

  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  private final BaseStatusSignal[] signals;

  private final Debouncer gyroConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  public GyroIOPigeon2() {
    m_Gyro = new Pigeon2(DriveConstants.DrivetrainConstants.Pigeon2Id, DriveConstants.kCANBus);

    Pigeon2Configuration gyroConfig = DriveConstants.DrivetrainConstants.Pigeon2Configs;
    m_Gyro.getConfigurator().apply(gyroConfig);
    m_Gyro
        .getConfigurator()
        .setYaw(
            DriverStation.getAlliance().get() == Alliance.Blue
                ? Radians.of(0)
                : Radians.of(Math.PI));

    yawSignal = m_Gyro.getYaw();
    pitchSignal = m_Gyro.getPitch();
    rollSignal = m_Gyro.getRoll();
    yawVelocitySignal = m_Gyro.getAngularVelocityZWorld();
    pitchVelocitySignal = m_Gyro.getAngularVelocityYWorld();
    rollVelocitySignal = m_Gyro.getAngularVelocityXWorld();
    robotXAcelerationSignal = m_Gyro.getAccelerationX();
    robotYAcelerationSignal = m_Gyro.getAccelerationY();

    signals = new BaseStatusSignal[] {yawSignal, yawVelocitySignal};

    yawSignal.setUpdateFrequency(RobotConstants.kServeOdometryFrequency);
    yawVelocitySignal.setUpdateFrequency(RobotConstants.kSwerveSignalFrequency);

    m_Gyro.optimizeBusUtilization();

    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_Gyro.getYaw());
  }

  @Override
  public void readInputs(GyroInputs inputs) {
    StatusCode gyroStatus = BaseStatusSignal.refreshAll(signals);

    inputs.gyroConnected = gyroConnectedDebouncer.calculate(gyroStatus.isOK());
    inputs.gyroYaw = yawSignal.getValue();
    inputs.gyroYawVelocity = yawVelocitySignal.getValue();
    inputs.gyroPitch = pitchSignal.getValue();
    inputs.gyroPitchVelocity = pitchVelocitySignal.getValue();
    inputs.gyroRoll = rollSignal.getValue();
    inputs.gyroRollVelocity = rollVelocitySignal.getValue();
    inputs.robotXAceleration = robotXAcelerationSignal.getValue();
    inputs.robotYAceleration = robotYAcelerationSignal.getValue();

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void resetGyro() {
    m_Gyro.setYaw(
        DriverStation.getAlliance().get() == Alliance.Blue ? Radians.of(0) : Radians.of(Math.PI));
  }
}
