package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.Constants.StateConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class RobotState {
  private static final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          new SwerveDriveKinematics(SwerveConstants.moduleTranslations),
          Rotation2d.kZero,
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          },
          Pose2d.kZero,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, 9999999));

  private static final TimeInterpolatableBuffer<Pose3d> fieldSpaceRobotPose =
      TimeInterpolatableBuffer.createBuffer(StateConstants.kBufferHistorySizeSeconds);
  private static final TimeInterpolatableBuffer<Double> robotYaw =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private static final TimeInterpolatableBuffer<Double> robotPitch =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private static final TimeInterpolatableBuffer<Double> robotRoll =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private static final TimeInterpolatableBuffer<Double> robotYawAngularVelocity =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private static final TimeInterpolatableBuffer<Double> robotRollAngularVelocity =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private static final TimeInterpolatableBuffer<Double> robotPitchAngularVelocity =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private static final TimeInterpolatableBuffer<Double> robotXAceleration =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private static final TimeInterpolatableBuffer<Double> robotYAceleration =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private static final TimeInterpolatableBuffer<Pose2d> visionPoses =
      TimeInterpolatableBuffer.createBuffer(StateConstants.kBufferHistorySizeSeconds);

  private static final AtomicReference<ChassisSpeeds> robotRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private static final AtomicReference<ChassisSpeeds> fieldRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private static final AtomicReference<ChassisSpeeds> robotRelativeChassisSpeedsSetpoint =
      new AtomicReference<>(new ChassisSpeeds());
  private static final AtomicReference<ChassisSpeeds> fieldRelativeChassisSpeedsSetpoint =
      new AtomicReference<>(new ChassisSpeeds());

  public static void addDriveMeasurements(
      double[] timestamps,
      SwerveModulePosition[] modulePositions,
      Rotation2d gyroAngles[],
      ChassisSpeeds mesuredFieldRelativeChassisSpeeds,
      ChassisSpeeds mesuredRobotRelativeChassisSpeeds,
      Angle robotRoll,
      Angle robotPitch,
      AngularVelocity pitchAngularVelocity,
      AngularVelocity rollAngularVelocity,
      LinearAcceleration xAceleration,
      LinearAcceleration yAceleration) {

    if (timestamps.length == 0) return;
    for (int i = 0; i < timestamps.length; i++) {
      poseEstimator.updateWithTime(timestamps[i], gyroAngles[i], modulePositions);
      RobotState.robotYaw.addSample(timestamps[i], gyroAngles[i].getRadians());

      addFieldSpaceRobotPose(timestamps[i], new Pose3d(poseEstimator.getEstimatedPosition()));
    }
    RobotState.fieldRelativeChassisSpeeds.set(mesuredFieldRelativeChassisSpeeds);
    RobotState.robotRelativeChassisSpeeds.set(mesuredRobotRelativeChassisSpeeds);

    double recentTimestamp = timestamps[timestamps.length - 1];
    RobotState.robotPitch.addSample(recentTimestamp, robotPitch.in(Radians));
    RobotState.robotRoll.addSample(recentTimestamp, robotRoll.in(Radians));

    RobotState.robotYawAngularVelocity.addSample(
        recentTimestamp, robotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
    RobotState.robotRollAngularVelocity.addSample(
        recentTimestamp, rollAngularVelocity.in(RadiansPerSecond));
    RobotState.robotPitchAngularVelocity.addSample(
        recentTimestamp, pitchAngularVelocity.in(RadiansPerSecond));
    RobotState.robotXAceleration.addSample(recentTimestamp, xAceleration.in(MetersPerSecondPerSecond));
    RobotState.robotYAceleration.addSample(recentTimestamp, yAceleration.in(MetersPerSecondPerSecond));
  }

  public static Pose2d getPredictedFieldSpaceRobotPose() {
    // TODO
    return Pose2d.kZero;
  }

  public static void addVisionMeasurements(PoseObservation observation) {
    visionPoses.addSample(observation.timestamp(), observation.pose().toPose2d());

    poseEstimator.addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp(), observation.stddevs());
  }

  public static void addFieldSpaceRobotPose(double timestamp, Pose3d pose) {
    fieldSpaceRobotPose.addSample(timestamp, pose);
  }

  public static void resetPose(Pose2d pose, SwerveModulePosition[] modulePositions) {
    poseEstimator.resetPosition(getLatestYaw(), modulePositions, pose);
  }

  // TODO
  // Add getters and manual setters for all buffers and speeds

  public static ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return robotRelativeChassisSpeeds.get();
  }

  public static ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return fieldRelativeChassisSpeeds.get();
  }

  public static ChassisSpeeds getRobotRelativeChassisSpeedsSetpoint() {
    return robotRelativeChassisSpeedsSetpoint.get();
  }

  public static ChassisSpeeds getFieldRelativeChassisSpeedsSetpoint() {
    return fieldRelativeChassisSpeedsSetpoint.get();
  }

  public static Pose2d getLatestPose2d() {
    return fieldSpaceRobotPose.getInternalBuffer().lastEntry().getValue().toPose2d();
  }

  private static Rotation2d getLatestYaw() {
    var lastEntry = robotYaw.getInternalBuffer().lastEntry();
    if (lastEntry == null) {
      return Rotation2d.kZero;
    }
    return Rotation2d.fromRadians(lastEntry.getValue());
  }
  
  public static Pose2d getLatestVisionPose() {
    var lastEntry = visionPoses.getInternalBuffer().lastEntry();
    if (lastEntry == null) {
      return new Pose2d();
    }
    return lastEntry.getValue();
  }

  public static void updateLogger() {
    Logger.recordOutput("RobotState/Odometry/Pose2D", poseEstimator.getEstimatedPosition());
    Logger.recordOutput(
        "RobotState/SwerveChassisSpeeds/FieldRelativeChassisSpeeds",
        getFieldRelativeChassisSpeeds());
    Logger.recordOutput(
        "RobotState/SwerveChassisSpeeds/RobotRelativeChassisSpeeds",
        getRobotRelativeChassisSpeeds());
    Logger.recordOutput("RobotState/VisionPoseEstimate", getLatestVisionPose());
  }

  // ====================================================================
  // ====================================================================
  //                     SEASON SPECIFIC STUFF
  // ====================================================================
  // ====================================================================
}
