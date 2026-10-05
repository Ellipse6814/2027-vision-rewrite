package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Util.SparkUtil.ifOk;
import static frc.robot.Util.SparkUtil.sparkStickyFault;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.RobotConstants;
import frc.robot.Util.SparkUtil;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ModuleIOSparkMax implements ModuleIO {

  private final SparkFlex driveMotor;
  private final SparkFlex steerMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder steerEncoder;

  private final CANcoder absoluteEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController steerController;

  private final StatusSignal<Angle> absoluteEncoderPositionSignal;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final Debouncer driveDebouncer;
  private final Debouncer turnDebouncer;
  private final Debouncer turnEncoderConnectedDebounce;
  private int id;

  public ModuleIOSparkMax(int id) {

    this.id = id;

    driveDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    turnDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    turnEncoderConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    driveMotor =
        new SparkFlex(
            switch (id) {
              case 0 -> SparkDriveConstants.kFrontLeftDriveMotorPort;
              case 1 -> SparkDriveConstants.kFrontRightDriveMotorPort;
              case 2 -> SparkDriveConstants.kBackLeftDriveMotorPort;
              case 3 -> SparkDriveConstants.kBackRightDriveMotorPort;
              default -> throw new IllegalArgumentException("Invalid module ID: " + id);
            },
            MotorType.kBrushless);

    steerMotor =
        new SparkFlex(
            switch (id) {
              case 0 -> SparkDriveConstants.kFrontLeftTurningMotorPort;
              case 1 -> SparkDriveConstants.kFrontRightTurningMotorPort;
              case 2 -> SparkDriveConstants.kBackLeftTurningMotorPort;
              case 3 -> SparkDriveConstants.kBackRightTurningMotorPort;
              default -> throw new IllegalArgumentException("Invalid module ID: " + id);
            },
            MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    steerEncoder = steerMotor.getEncoder();

    driveController = driveMotor.getClosedLoopController();
    steerController = steerMotor.getClosedLoopController();

    absoluteEncoder =
        new CANcoder(
            switch (id) {
              case 0 -> SparkDriveConstants.kFrontLeftAbsoluteEncoderPort;
              case 1 -> SparkDriveConstants.kFrontRightAbsoluteEncoderPort;
              case 2 -> SparkDriveConstants.kBackLeftAbsoluteEncoderPort;
              case 3 -> SparkDriveConstants.kBackRightAbsoluteEncoderPort;
              default -> throw new IllegalArgumentException("Invalid module ID: " + id);
            });

    CANcoderConfiguration absoluteEncoderConfig = new CANcoderConfiguration();
    absoluteEncoderConfig.MagnetSensor.MagnetOffset =
        switch (id) {
          case 0 -> SparkDriveConstants.kFrontLeftAbsoluteEncoderOffset;
          case 1 -> SparkDriveConstants.kFrontRightAbsoluteEncoderOffset;
          case 2 -> SparkDriveConstants.kBackLeftAbsoluteEncoderOffset;
          case 3 -> SparkDriveConstants.kBackRightAbsoluteEncoderOffset;
          default -> throw new IllegalArgumentException("Invalid module ID: " + id);
        };
    absoluteEncoderConfig.MagnetSensor.SensorDirection =
        switch (id) {
          case 0 -> SparkDriveConstants.kFrontLeftAbsoluteEncoderReversed
              ? SensorDirectionValue.Clockwise_Positive
              : SensorDirectionValue.CounterClockwise_Positive;
          case 1 -> SparkDriveConstants.kFrontRightAbsoluteEncoderReversed
              ? SensorDirectionValue.Clockwise_Positive
              : SensorDirectionValue.CounterClockwise_Positive;
          case 2 -> SparkDriveConstants.kBackLeftAbsoluteEncoderReversed
              ? SensorDirectionValue.Clockwise_Positive
              : SensorDirectionValue.CounterClockwise_Positive;
          case 3 -> SparkDriveConstants.kBackRightAbsoluteEncoderReversed
              ? SensorDirectionValue.Clockwise_Positive
              : SensorDirectionValue.CounterClockwise_Positive;
          default -> throw new IllegalArgumentException("Invalid module ID: " + id);
        };

    SparkFlexConfig driveConfig = new SparkFlexConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(SparkDriveConstants.kDriveCurrentLimit)
        .voltageCompensation(12)
        .inverted(
            switch (id) {
              case 0 -> SparkDriveConstants.kFrontLeftDriveEncoderReversed;
              case 1 -> SparkDriveConstants.kFrontRightDriveEncoderReversed;
              case 2 -> SparkDriveConstants.kBackLeftDriveEncoderReversed;
              case 3 -> SparkDriveConstants.kBackRightDriveEncoderReversed;
              default -> throw new IllegalArgumentException("Invalid module ID: " + id);
            });
    driveConfig
        .encoder
        .positionConversionFactor(SparkDriveConstants.kDriveRotationstoRadians)
        .velocityConversionFactor(SparkDriveConstants.kDriveRPMtoRadiansPerSecond)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            SparkDriveConstants.kDriveKP,
            SparkDriveConstants.kDriveKI,
            SparkDriveConstants.kDriveKD);
    // .feedForward
    // .sva(SparkDriveConstants.kDriveKS, SparkDriveConstants.kDriveKV,
    // SparkDriveConstants.kDriveKA);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(
            (int) (1000.0 / RobotConstants.kServeOdometryFrequency.in(Hertz)))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    SparkUtil.tryUntilOk(
        driveMotor,
        5,
        () ->
            driveMotor.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    SparkUtil.tryUntilOk(driveMotor, 5, () -> driveEncoder.setPosition(0.0));

    SparkFlexConfig steerConfig = new SparkFlexConfig();
    steerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(SparkDriveConstants.kDriveCurrentLimit)
        .voltageCompensation(12)
        .inverted(
            switch (id) {
              case 0 -> SparkDriveConstants.kFrontLeftTurningEncoderReversed;
              case 1 -> SparkDriveConstants.kFrontRightTurningEncoderReversed;
              case 2 -> SparkDriveConstants.kBackLeftTurningEncoderReversed;
              case 3 -> SparkDriveConstants.kBackRightTurningEncoderReversed;
              default -> throw new IllegalArgumentException("Invalid module ID: " + id);
            });
    steerConfig
        .encoder
        .positionConversionFactor(SparkDriveConstants.kTurningRotationstoRadians)
        .velocityConversionFactor(SparkDriveConstants.kTurningRPMtoRadiansPerSecond)
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);
    steerConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            SparkDriveConstants.kTurningKP,
            SparkDriveConstants.kTurningKI,
            SparkDriveConstants.kTurningKD)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 2 * Math.PI);
    // .feedForward
    // .sva(SparkDriveConstants.kTurningKS, SparkDriveConstants.kTurningKV,
    // SparkDriveConstants.kTurningKA);
    steerConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(
            (int) (1000.0 / RobotConstants.kServeOdometryFrequency.in(Hertz)))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    SparkUtil.tryUntilOk(
        steerMotor,
        5,
        () ->
            steerMotor.configure(
                steerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    SparkUtil.tryUntilOk(
        steerMotor,
        5,
        () -> steerEncoder.setPosition(absoluteEncoder.getPosition().getValue().in(Radians)));

    absoluteEncoderPositionSignal = absoluteEncoder.getPosition();

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(steerEncoder::getPosition);
  }

  @Override
  public void readInputs(ModuleInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(
        driveMotor, driveEncoder::getPosition, (value) -> inputs.drivePosition = Radians.of(value));
    ifOk(
        driveMotor,
        driveEncoder::getVelocity,
        (value) -> inputs.driveVelocity = RadiansPerSecond.of(value));
    ifOk(
        driveMotor,
        new DoubleSupplier[] {driveMotor::getAppliedOutput, driveMotor::getBusVoltage},
        (values) -> inputs.driveAppliedVoltage = Volts.of(values[0] * values[1]));
    ifOk(
        driveMotor,
        driveMotor::getOutputCurrent,
        (value) -> inputs.driveTorqueCurrent = Amps.of(value));
    inputs.driveConnected = driveDebouncer.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(steerMotor, steerEncoder::getPosition, (value) -> inputs.turnPosition = Radians.of(value));
    ifOk(
        steerMotor,
        steerEncoder::getVelocity,
        (value) -> inputs.turnVelocity = RadiansPerSecond.of(value));
    ifOk(
        steerMotor,
        new DoubleSupplier[] {steerMotor::getAppliedOutput, steerMotor::getBusVoltage},
        (values) -> inputs.turnAppliedVoltage = Volts.of(values[0] * values[1]));
    ifOk(
        steerMotor,
        steerMotor::getOutputCurrent,
        (value) -> inputs.turnStatorCurrent = Amps.of(value));
    inputs.turnConnected = turnDebouncer.calculate(!sparkStickyFault);

    StatusCode encoderStatus = BaseStatusSignal.refreshAll(absoluteEncoderPositionSignal);

    inputs.absouluteEncoderConnected = turnEncoderConnectedDebounce.calculate(encoderStatus.isOK());
    inputs.absouluteTurnPosition = absoluteEncoderPositionSignal.getValue();

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void driveOpenLoop(double output) {
    driveMotor.setVoltage(output);
  }

  @Override
  public void turnOpenLoop(double output) {
    steerMotor.setVoltage(output);
  }

  @Override
  public void driveClosedLoop(AngularVelocity output) {
    Logger.recordOutput("Drive/DriveSetpoint " + id, output);
    driveController.setSetpoint(output.in(RadiansPerSecond), ControlType.kVelocity);
  }

  @Override
  public void turnClosedLoop(Rotation2d output) {
    steerController.setSetpoint(output.getRadians(), ControlType.kPosition);
  }
}
