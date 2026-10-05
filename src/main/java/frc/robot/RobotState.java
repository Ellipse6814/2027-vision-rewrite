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

  private static RobotState instance;

  public static RobotState getRobotState() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private RobotState() {}

  private final SwerveDrivePoseEstimator poseEstimator =
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

  private final TimeInterpolatableBuffer<Pose3d> fieldSpaceRobotPose =
      TimeInterpolatableBuffer.createBuffer(StateConstants.kBufferHistorySizeSeconds);
  private final TimeInterpolatableBuffer<Double> robotYaw =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private final TimeInterpolatableBuffer<Double> robotPitch =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private final TimeInterpolatableBuffer<Double> robotRoll =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private final TimeInterpolatableBuffer<Double> robotYawAngularVelocity =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private final TimeInterpolatableBuffer<Double> robotRollAngularVelocity =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private final TimeInterpolatableBuffer<Double> robotPitchAngularVelocity =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private final TimeInterpolatableBuffer<Double> robotXAceleration =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private final TimeInterpolatableBuffer<Double> robotYAceleration =
      TimeInterpolatableBuffer.createDoubleBuffer(StateConstants.kBufferHistorySizeSeconds);
  private final TimeInterpolatableBuffer<Pose2d> visionPoses =
      TimeInterpolatableBuffer.createBuffer(StateConstants.kBufferHistorySizeSeconds);

  private final AtomicReference<ChassisSpeeds> robotRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> fieldRelativeChassisSpeeds =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> robotRelativeChassisSpeedsSetpoint =
      new AtomicReference<>(new ChassisSpeeds());
  private final AtomicReference<ChassisSpeeds> fieldRelativeChassisSpeedsSetpoint =
      new AtomicReference<>(new ChassisSpeeds());

  public void addDriveMeasurements(
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
      this.robotYaw.addSample(timestamps[i], gyroAngles[i].getRadians());

      addFieldSpaceRobotPose(timestamps[i], new Pose3d(poseEstimator.getEstimatedPosition()));
    }
    this.fieldRelativeChassisSpeeds.set(mesuredFieldRelativeChassisSpeeds);
    this.robotRelativeChassisSpeeds.set(mesuredRobotRelativeChassisSpeeds);

    double recentTimestamp = timestamps[timestamps.length - 1];
    this.robotPitch.addSample(recentTimestamp, robotPitch.in(Radians));
    this.robotRoll.addSample(recentTimestamp, robotRoll.in(Radians));

    this.robotYawAngularVelocity.addSample(
        recentTimestamp, robotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
    this.robotRollAngularVelocity.addSample(
        recentTimestamp, rollAngularVelocity.in(RadiansPerSecond));
    this.robotPitchAngularVelocity.addSample(
        recentTimestamp, pitchAngularVelocity.in(RadiansPerSecond));
    this.robotXAceleration.addSample(recentTimestamp, xAceleration.in(MetersPerSecondPerSecond));
    this.robotYAceleration.addSample(recentTimestamp, yAceleration.in(MetersPerSecondPerSecond));
  }

  public Pose2d getPredictedFieldSpaceRobotPose() {
    // TODO
    return Pose2d.kZero;
  }

  public void addVisionMeasurements(PoseObservation observation, Vector<N3> stdDevs) {
    visionPoses.addSample(observation.timestamp(), observation.pose().toPose2d());
    poseEstimator.addVisionMeasurement(observation.pose().toPose2d(), observation.timestamp());
    // System.out.println(
    //     "Added vision measurement: "
    //         + observation.pose().toPose2d().toString()
    //         + " at time "
    //         + observation.timestamp());
  }

  public void addFieldSpaceRobotPose(double timestamp, Pose3d pose) {
    fieldSpaceRobotPose.addSample(timestamp, pose);
  }

  public void resetPose(Pose2d pose, SwerveModulePosition[] modulePositions) {
    poseEstimator.resetPosition(getLatestYaw(), modulePositions, pose);
  }

  // TODO
  // Add getters and manual setters for all buffers and speeds

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return robotRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return fieldRelativeChassisSpeeds.get();
  }

  public ChassisSpeeds getRobotRelativeChassisSpeedsSetpoint() {
    return robotRelativeChassisSpeedsSetpoint.get();
  }

  public ChassisSpeeds getFieldRelativeChassisSpeedsSetpoint() {
    return fieldRelativeChassisSpeedsSetpoint.get();
  }

  public Pose2d getLatestPose2d() {
    return fieldSpaceRobotPose.getInternalBuffer().lastEntry().getValue().toPose2d();
  }

  private Rotation2d getLatestYaw() {
    var lastEntry = robotYaw.getInternalBuffer().lastEntry();
    if (lastEntry == null) {
      return Rotation2d.kZero;
    }
    return Rotation2d.fromRadians(lastEntry.getValue());
  }

  public void updateLogger() {
    Logger.recordOutput("RobotState/Odometry/Pose2D", poseEstimator.getEstimatedPosition());
    Logger.recordOutput(
        "RobotState/SwerveChassisSpeeds/FieldRelativeChassisSpeeds",
        getFieldRelativeChassisSpeeds());
    Logger.recordOutput(
        "RobotState/SwerveChassisSpeeds/RobotRelativeChassisSpeeds",
        getRobotRelativeChassisSpeeds());
    Logger.recordOutput("RobotState/VisionPoseEstimate", getLatestVisionPose());

    // Game specific
    Logger.recordOutput("RobotState/Turret/FieldSpaceYaw", getTurretFieldSpaceYaw());
    Logger.recordOutput("RobotState/Turret/RobotSpaceYaw", getTurretRobotSpaceYaw());
    Logger.recordOutput("RobotState/Turret/FieldSpaceVelocity", getTurretFieldRelativeVelocity());
    Logger.recordOutput("RobotState/Turret/RobotSpaceVelocity", getTurretRobotRelativeVelocity());
  }

  // Game specific
  private Rotation2d turretRobotRelativeYaw = Rotation2d.kZero;
  private Rotation2d turretFieldRelativeYaw = Rotation2d.kZero;
  private AngularVelocity turretRobotRelativeVelocity = RadiansPerSecond.of(0);
  private AngularVelocity turretFieldRelativeVelocity = RadiansPerSecond.of(0);
  private boolean isTurretAtSetpoint = true;

  public void addTurretMeasurements(
      Angle robotRelativeYaw, AngularVelocity robotRelativeVelocity, boolean isAtSetpoint) {
    turretRobotRelativeYaw = Rotation2d.fromRadians(robotRelativeYaw.in(Radian));
    turretFieldRelativeYaw = getLatestYaw().plus(turretRobotRelativeYaw);
    isTurretAtSetpoint = isAtSetpoint;

    turretRobotRelativeVelocity = robotRelativeVelocity;
    turretFieldRelativeVelocity =
        robotRelativeVelocity.plus(
            RadiansPerSecond.of(getFieldRelativeChassisSpeeds().omegaRadiansPerSecond));
  }

  public Pose2d getLatestVisionPose() {
    var lastEntry = visionPoses.getInternalBuffer().lastEntry();
    if (lastEntry == null) {
      return new Pose2d();
    }
    return lastEntry.getValue();
  }

  public Rotation2d getTurretFieldSpaceYaw() {
    return turretFieldRelativeYaw;
  }

  public Rotation2d getTurretRobotSpaceYaw() {
    return turretRobotRelativeYaw;
  }

  public AngularVelocity getTurretRobotRelativeVelocity() {
    return turretRobotRelativeVelocity.copy();
  }

  public AngularVelocity getTurretFieldRelativeVelocity() {
    return turretFieldRelativeVelocity.copy();
  }

  public boolean isTurretAtSetpoint() {
    return isTurretAtSetpoint;
  }
}
