package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

  @AutoLog
  public class GyroInputs {
    public boolean gyroConnected = false;
    public Angle gyroYaw = Radians.of(0);
    public Angle gyroPitch = Radians.of(0);
    public Angle gyroRoll = Radians.of(0);
    public AngularVelocity gyroYawVelocity = RadiansPerSecond.of(0);
    public AngularVelocity gyroPitchVelocity = RadiansPerSecond.of(0);
    public AngularVelocity gyroRollVelocity = RadiansPerSecond.of(0);
    public LinearAcceleration robotXAceleration = MetersPerSecondPerSecond.of(0);
    public LinearAcceleration robotYAceleration = MetersPerSecondPerSecond.of(0);
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void readInputs(GyroInputs inputs) {}

  public default void resetGyro() {}
}
