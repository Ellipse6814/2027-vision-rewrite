package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.Logger;

public class Module {

  private final ModuleIO IO;
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(
      ModuleIO IO,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.IO = IO;
    this.index = index;
    this.constants = constants;
  }

  public void periodic() {
    IO.readInputs(inputs);
    Logger.processInputs("Drive/Module/" + index, inputs);

    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  public void setDesiredState(SwerveModuleState state) {
    state.optimize(getAngle());
    state.cosineScale(Rotation2d.fromRadians(inputs.absouluteTurnPosition.in(Radians)));

    AngularVelocity moduleSpeed =
        RadiansPerSecond.of(state.speedMetersPerSecond / constants.WheelRadius);
    Rotation2d moduleAngle = state.angle;

    IO.driveClosedLoop(moduleSpeed);
    IO.turnClosedLoop(moduleAngle);
  }

  public void stop() {
    IO.driveOpenLoop(0.0);
    IO.turnOpenLoop(0.0);
  }

  public void runOutput(double driveOutput, double turnOutput) {
    IO.driveOpenLoop(driveOutput);
    IO.turnOpenLoop(turnOutput);
  }

  public void runCharacterization(double output) {
    IO.driveOpenLoop(output);
    IO.turnClosedLoop(Rotation2d.kZero);
  }

  public Distance getPosition() {
    Distance position =
        Meters.of(
            inputs.drivePosition.in(Radians)
                * constants.WheelRadius
                * (1 / constants.DriveMotorGearRatio));
    return position;
  }

  public LinearVelocity getVelocity() {
    LinearVelocity velocity =
        MetersPerSecond.of(
            inputs.driveVelocity.in(RadiansPerSecond)
                * constants.WheelRadius
                * (1 / constants.DriveMotorGearRatio));
    return velocity;
  }

  public AngularVelocity getOmega() {
    return inputs.driveVelocity;
  }

  public SwerveModuleState getState() {
    SwerveModuleState state = new SwerveModuleState(getVelocity(), getAngle());
    return state;
  }

  public SwerveModulePosition getSwervePosition() {
    SwerveModulePosition swervePosition = new SwerveModulePosition(getPosition(), getAngle());
    return swervePosition;
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public Rotation2d getAngle() {
    return new Rotation2d(inputs.turnPosition);
  }

  public Rotation2d getDriveAngle() {
    return new Rotation2d(getPosition().in(Meters) / constants.WheelRadius);
  }

  public int getIndex() {
    return index;
  }
}
