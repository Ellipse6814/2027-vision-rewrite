package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  public class ModuleInputs {
    public boolean driveConnected = false;
    public Angle drivePosition = Radians.of(0);
    public AngularVelocity driveVelocity = RadiansPerSecond.of(0);
    public Voltage driveAppliedVoltage = Volts.of(0);
    public Voltage driveSupplyVoltage = Volts.of(0);
    public Current driveTorqueCurrent = Amps.of(0);
    public Current driveSupplyCurrent = Amps.of(0);

    public boolean turnConnected = false;
    public Angle turnPosition = Radians.of(0);
    public AngularVelocity turnVelocity = RadiansPerSecond.of(0);
    public Voltage turnAppliedVoltage = Volts.of(0);
    public Voltage turnSupplyVoltage = Volts.of(0);
    public Current turnStatorCurrent = Amps.of(0);
    public Current turnSupplyCurrent = Amps.of(0);

    public boolean absouluteEncoderConnected = false;
    public Angle absouluteTurnPosition = Radians.of(0);

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

    public double driveClosedLoopReference = 0;
    public double kDriveClosedLoopOutput = 0;
  }

  public default void readInputs(ModuleInputs inputs) {}

  public default void driveOpenLoop(double output) {}

  public default void turnOpenLoop(double output) {}

  public default void driveClosedLoop(AngularVelocity output) {}

  public default void turnClosedLoop(Rotation2d output) {}
}
