package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.DriveConstants;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class Constants {
  // Motor IDs:
  // 10-19 turret/shooter/hood
  // 20-29 indexer
  // 30-39 intake
  // 40-49 climb

  public static final Mode simMode = Mode.REPLAY;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static class FieldConstants {
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public static final double fieldWidth = 8.069326;
    public static final double fieldLength = 16.540988;

    public static final Distance kFieldMaxX = Meters.of(16.540988);
    public static final Distance kFieldMaxY = Meters.of(8.069326);
  }

  public static class RobotConstants {

    public static final Mass kRobotMass = Pounds.of(129.4);
    public static final MomentOfInertia kMOI = KilogramSquareMeters.of(6.883);

    public static final Frequency kServeOdometryFrequency = Hertz.of(200);
    public static final Frequency kSwerveSignalFrequency = Hertz.of(50);

    public static final double kWheelCOF = 1.2;
  }

  public static class OIConstants {
    public static final int kDriveContollerPort = 0;
    public static final int kTechContollerPort = 1;

    public static double kDeadband = 0.1;

    public static double kThetaControllerkP = 5.0;
    public static double KThetaControllerkI = 0.0;
    public static double kThetaControllerkD = 0.5;

    public static AngularVelocity kThetaControllerMaxVelocity = RadiansPerSecond.of(0.0);
    public static AngularAcceleration kThetaControllerMaxAcceleration =
        RadiansPerSecondPerSecond.of(0.0);

    public static TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kThetaControllerMaxVelocity.in(RadiansPerSecond),
            kThetaControllerMaxAcceleration.in(RadiansPerSecondPerSecond));
  }

  public static class SwerveConstants {
    public static final AngularVelocity kMaxSteerVelocity = RotationsPerSecond.of(10.0);

    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
          new Translation2d(
              DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY),
          new Translation2d(DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
          new Translation2d(DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY)
        };

    public static final Distance kDriveBaseRadius =
        Meter.of(
            Math.max(
                Math.max(
                    Math.hypot(
                        DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
                    Math.hypot(
                        DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY)),
                Math.max(
                    Math.hypot(
                        DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
                    Math.hypot(
                        DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY))));

    public static final AngularVelocity kAngularSpeedAt12Volts =
        RadiansPerSecond.of(
            DriveConstants.kSpeedAt12Volts.in(MetersPerSecond) / kDriveBaseRadius.in(Meters));

    public static final ModuleConfig kModuleConfig =
        new ModuleConfig(
            DriveConstants.FrontLeft.WheelRadius,
            DriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
            RobotConstants.kWheelCOF,
            DCMotor.getFalcon500Foc(1).withReduction(DriveConstants.FrontLeft.DriveMotorGearRatio),
            DriveConstants.FrontLeft.SlipCurrent,
            1);

    public static final SwerveModuleSimulationConfig kModuleSimConfig =
        new SwerveModuleSimulationConfig(
            DCMotor.getFalcon500Foc(1),
            DCMotor.getFalcon500Foc(1),
            DriveConstants.FrontLeft.DriveMotorGearRatio,
            DriveConstants.FrontLeft.SteerMotorGearRatio,
            Volts.of(DriveConstants.FrontLeft.DriveFrictionVoltage),
            Volts.of(DriveConstants.FrontLeft.SteerFrictionVoltage),
            Meters.of(DriveConstants.FrontLeft.WheelRadius),
            KilogramSquareMeters.of(DriveConstants.FrontLeft.SteerInertia),
            RobotConstants.kWheelCOF);

    public static final RobotConfig kRobotConfig =
        new RobotConfig(
            RobotConstants.kRobotMass, RobotConstants.kMOI, kModuleConfig, moduleTranslations);

    public static final DriveTrainSimulationConfig driveSimConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(RobotConstants.kRobotMass)
            .withCustomModuleTranslations(moduleTranslations)
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(kModuleSimConfig);

    public static class CharacterizationConstants {
      public static final double kFFStartDelay = 2.0; // Secs
      public static final double kFFRampRate = 0.1; // Volts/Sec
      public static final double kWheelRadiusMaxVelocity = 0.25; // Rad/Sec
      public static final double kWheelRadiusRampRate = 0.05; // Rad/Sec^2
    }

    public static class PathfindingConstraints {
      public static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(6.0);
      public static final AngularAcceleration maxAngularAcceleration =
          RadiansPerSecondPerSecond.of(12.0);
      public static final LinearVelocity maxVelocity = MetersPerSecond.of(4);
      public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(5);
    }
  }

  public static class StateConstants {
    public static final double kBufferHistorySizeSeconds = 1.0;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class VisionConstants {
    public static final String kCameraAName = "limelight-main";

    public static final Distance kCameraAMountingX = Inches.of(3.75);
    public static final Distance kCameraAMountingY = Inches.of(1.0);
    public static final Distance kCameraAMountingZ = Inches.of(18.77);
    public static final Angle kCameraAMountingYaw = Degrees.of(0.0);
    public static final Angle kCameraAMountingPitch = Degrees.of(9.2);
    public static final Angle kCameraAMountingRoll = Degrees.of(0.0);

    public static final Distance kMaxPoseError = Meters.of(68.14);
    public static final AngularVelocity kMaxCameraOmega = RotationsPerSecond.of(3.0);

    public static final double kLinearStdDevBaseline = 0.5;
    public static final double kAngularStdDevBaseline = Double.POSITIVE_INFINITY;
  }
}
