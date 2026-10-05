package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Util.PhoenixUtil;

public class ModuleIOTalonFX implements ModuleIO {

  public TalonFX m_Drive;
  public TalonFX m_Turn;
  public CANcoder m_AbsouluteEncoder;

  public VoltageOut voltageControl = new VoltageOut(Volts.of(0));
  public VelocityVoltage velocityVoltageControl = new VelocityVoltage(RadiansPerSecond.of(0));
  public PositionVoltage positionVoltageControl = new PositionVoltage(Radians.of(0));

  public TorqueCurrentFOC torqueCurrentControl = new TorqueCurrentFOC(Amps.of(0));
  public VelocityTorqueCurrentFOC velocityTorqueCurrentControl =
      new VelocityTorqueCurrentFOC(RadiansPerSecond.of(0));
  public PositionTorqueCurrentFOC positionTorqueCurrentControl =
      new PositionTorqueCurrentFOC(Radians.of(0));

  SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  public StatusSignal<Angle> drivePositionSignal;
  public StatusSignal<AngularVelocity> driveVelocitySignal;
  public StatusSignal<Voltage> driveAppliedVoltageSignal;
  public StatusSignal<Voltage> driveSupplyVoltageSignal;
  public StatusSignal<Current> driveTorqueCurrentSignal;
  public StatusSignal<Current> driveSupplyCurrentSignal;

  public StatusSignal<Angle> turnPositionSignal;
  public StatusSignal<AngularVelocity> turnVelocitySignal;
  public StatusSignal<Voltage> turnAppliedVoltageSignal;
  public StatusSignal<Voltage> turnSupplyVoltageSignal;
  public StatusSignal<Current> turnStatorCurrentSignal;
  public StatusSignal<Current> turnSupplyCurrentSignal;

  public StatusSignal<Angle> absouluteTurnPositionSignal;

  public StatusSignal<Double> driveClosedLoopReferenceSignal;
  public StatusSignal<Double> driveClosedLoopOutputSignal;

  public BaseStatusSignal[] highFrequencySignals;
  public BaseStatusSignal[] standardSignals;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;

    m_Drive = new TalonFX(constants.DriveMotorId, DriveConstants.kCANBus);
    m_Turn = new TalonFX(constants.SteerMotorId, DriveConstants.kCANBus);
    m_AbsouluteEncoder = new CANcoder(constants.EncoderId, DriveConstants.kCANBus);

    // Config Drive Motor
    TalonFXConfiguration driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Config Turn Motor
    TalonFXConfiguration turnConfig = constants.SteerMotorInitialConfigs;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    if (Constants.currentMode == Constants.Mode.SIM)
      turnConfig.Slot0.withKD(0.5).withKS(0); // during simulation, gains are slightly differen
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    switch (constants.FeedbackSource) {
      case RemoteCANcoder:
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        break;
      case FusedCANcoder:
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        break;
      case SyncCANcoder:
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        break;
      default:
        throw new RuntimeException("No valid feedback sensor source.");
    }

    // CANCoder Config
    CANcoderConfiguration absouluteEncoderConfig = constants.EncoderInitialConfigs;
    absouluteEncoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    absouluteEncoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    // Configure Motors
    PhoenixUtil.tryUntilOk(5, () -> m_Drive.getConfigurator().apply(driveConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> m_Turn.getConfigurator().apply(turnConfig, 0.25));

    PhoenixUtil.tryUntilOk(5, () -> m_Drive.setPosition(0.0, 0.25));

    m_AbsouluteEncoder.getConfigurator().apply(absouluteEncoderConfig);

    // Handle Signals

    drivePositionSignal = m_Drive.getPosition();
    driveVelocitySignal = m_Drive.getVelocity();
    driveAppliedVoltageSignal = m_Drive.getMotorVoltage();
    driveSupplyVoltageSignal = m_Drive.getSupplyVoltage();
    driveTorqueCurrentSignal = m_Drive.getTorqueCurrent();
    driveSupplyCurrentSignal = m_Drive.getSupplyCurrent();

    turnPositionSignal = m_Turn.getPosition();
    turnVelocitySignal = m_Turn.getVelocity();
    turnAppliedVoltageSignal = m_Turn.getMotorVoltage();
    turnSupplyVoltageSignal = m_Turn.getSupplyVoltage();
    turnStatorCurrentSignal = m_Turn.getStatorCurrent();
    turnSupplyCurrentSignal = m_Turn.getSupplyCurrent();

    absouluteTurnPositionSignal = m_AbsouluteEncoder.getAbsolutePosition();

    driveClosedLoopReferenceSignal = m_Drive.getClosedLoopReference();
    driveClosedLoopOutputSignal = m_Drive.getClosedLoopOutput();

    highFrequencySignals = new BaseStatusSignal[] {drivePositionSignal, turnPositionSignal};

    standardSignals =
        new BaseStatusSignal[] {
          driveVelocitySignal,
          driveAppliedVoltageSignal,
          driveSupplyVoltageSignal,
          driveTorqueCurrentSignal,
          driveSupplyCurrentSignal,
          turnVelocitySignal,
          turnAppliedVoltageSignal,
          turnSupplyVoltageSignal,
          turnStatorCurrentSignal,
          turnSupplyCurrentSignal,
          absouluteTurnPositionSignal,
          driveClosedLoopReferenceSignal,
          driveClosedLoopOutputSignal
        };

    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotConstants.kServeOdometryFrequency, highFrequencySignals);
    BaseStatusSignal.setUpdateFrequencyForAll(
        RobotConstants.kSwerveSignalFrequency, standardSignals);

    ParentDevice.optimizeBusUtilizationForAll(m_Drive, m_Turn, m_AbsouluteEncoder);
  }

  @Override
  public void readInputs(ModuleInputs inputs) {
    StatusCode driveStatus =
        BaseStatusSignal.refreshAll(
            drivePositionSignal,
            driveVelocitySignal,
            driveAppliedVoltageSignal,
            driveSupplyVoltageSignal,
            driveTorqueCurrentSignal,
            driveSupplyCurrentSignal);

    StatusCode turnStatus =
        BaseStatusSignal.refreshAll(
            turnPositionSignal,
            turnVelocitySignal,
            turnAppliedVoltageSignal,
            turnSupplyVoltageSignal,
            turnStatorCurrentSignal,
            turnSupplyCurrentSignal);

    StatusCode encoderStatus = BaseStatusSignal.refreshAll(absouluteTurnPositionSignal);

    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePosition = drivePositionSignal.getValue();
    inputs.driveVelocity = driveVelocitySignal.getValue();
    inputs.driveAppliedVoltage = driveAppliedVoltageSignal.getValue();
    inputs.driveSupplyVoltage = driveSupplyVoltageSignal.getValue();
    inputs.driveTorqueCurrent = driveTorqueCurrentSignal.getValue();
    inputs.driveSupplyCurrent = driveSupplyCurrentSignal.getValue();

    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnPosition = turnPositionSignal.getValue();
    inputs.turnVelocity = turnVelocitySignal.getValue();
    inputs.turnAppliedVoltage = turnAppliedVoltageSignal.getValue();
    inputs.turnSupplyVoltage = turnSupplyVoltageSignal.getValue();
    inputs.turnStatorCurrent = turnStatorCurrentSignal.getValue();
    inputs.turnSupplyCurrent = turnSupplyCurrentSignal.getValue();

    inputs.absouluteEncoderConnected = turnEncoderConnectedDebounce.calculate(encoderStatus.isOK());
    inputs.absouluteTurnPosition = absouluteTurnPositionSignal.getValue();
  }

  @Override
  public void driveOpenLoop(double output) {
    switch (constants.DriveMotorClosedLoopOutput) {
      case Voltage:
        m_Drive.setControl(voltageControl.withOutput(output));
        break;
      case TorqueCurrentFOC:
        m_Drive.setControl(torqueCurrentControl.withOutput(output));
        break;
    }
  }

  @Override
  public void turnOpenLoop(double output) {
    switch (constants.SteerMotorClosedLoopOutput) {
      case Voltage:
        m_Turn.setControl(voltageControl.withOutput(output));
        break;
      case TorqueCurrentFOC:
        m_Turn.setControl(torqueCurrentControl.withOutput(output));
        break;
    }
  }

  @Override
  public void driveClosedLoop(AngularVelocity setpoint) {
    setpoint = setpoint.times(constants.DriveMotorGearRatio);
    switch (constants.DriveMotorClosedLoopOutput) {
      case Voltage:
        m_Drive.setControl(velocityVoltageControl.withVelocity(setpoint));
        break;

      case TorqueCurrentFOC:
        m_Drive.setControl(velocityTorqueCurrentControl.withVelocity(setpoint));
        break;
    }
  }

  @Override
  public void turnClosedLoop(Rotation2d setpoint) {
    switch (constants.SteerMotorClosedLoopOutput) {
      case Voltage:
        m_Turn.setControl(positionVoltageControl.withPosition(setpoint.getMeasure()));
        break;

      case TorqueCurrentFOC:
        m_Turn.setControl(positionTorqueCurrentControl.withPosition(setpoint.getMeasure()));
        break;
    }
  }
}
