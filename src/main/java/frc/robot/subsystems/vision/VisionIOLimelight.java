package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.Util.LimelightHelpers;
import java.util.ArrayList;
import java.util.List;

public class VisionIOLimelight implements VisionIO {

  private final DoubleSubscriber latencySub;
  private final DoubleArraySubscriber poseSub;

  private final String cameraName;
  private final Translation3d mountingTranslation;
  private final Rotation3d mountingRotation;

  public VisionIOLimelight(
      String cameraName, Translation3d mountingTranslation, Rotation3d mountingRotation) {
    this.cameraName = cameraName;
    this.mountingTranslation = mountingTranslation;
    this.mountingRotation = mountingRotation;

    NetworkTable nt = NetworkTableInstance.getDefault().getTable(cameraName);

    latencySub = nt.getDoubleTopic("tl").subscribe(0.0);
    poseSub = nt.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[6]);

    setCameraPosition();
  }

  @Override
  public void readInputs(AprilTagInputs inputs) {

    setRobotOrientation();
    inputs.cameraName = cameraName;
    inputs.cameraConnected = ((Timer.getFPGATimestamp() - latencySub.getLastChange()) / 1000) < 250;

    inputs.cameraHasTarget = LimelightHelpers.getTV(cameraName);

    List<PoseObservation> poseObservations = new ArrayList<>();
    for (var sample : poseSub.readQueue()) {
      if (sample.value.length == 0) continue;

      double[] stddevsArray = LimelightHelpers.getStdDevs(cameraName);
      Vector<N3> stddevsVec = VecBuilder.fill(stddevsArray[6], stddevsArray[7], stddevsArray[11]);

      poseObservations.add(
          new PoseObservation(
              (sample.timestamp * 1e-6) - (sample.value[6] * 1e-6),
              parsePose(sample.value),
              getCameraAngularVelocity(),
              0.0,
              (int) sample.value[7],
              sample.value[9],
              stddevsVec,
              PoseObservationType.MEGATAG_2));
    }

    inputs.poseObservations = poseObservations.toArray(PoseObservation[]::new);
  }

  @Override
  public void setCameraPosition() {
    LimelightHelpers.setCameraPose_RobotSpace(
        cameraName,
        mountingTranslation.getX(),
        -mountingTranslation.getY(),
        mountingTranslation.getZ(),
        Radians.of(mountingRotation.getX()).in(Degrees),
        Radians.of(mountingRotation.getY()).in(Degrees),
        Radians.of(mountingRotation.getZ()).in(Degrees));
  }

  @Override
  public void setRobotOrientation() {
    LimelightHelpers.SetRobotOrientation(
        cameraName,
        RobotState.getLatestPose2d().getRotation().getDegrees(),
        RadiansPerSecond.of(RobotState.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond)
            .in(DegreesPerSecond),
        0,
        0,
        0,
        0);
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Radians.of(rawLLArray[3]), Radians.of(rawLLArray[4]), Radians.of(rawLLArray[5])));
  }

  public String getCameraName() {
    return cameraName;
  }

  protected AngularVelocity getCameraAngularVelocity() {
    return RadiansPerSecond.of(RobotState.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond);
  }
}
