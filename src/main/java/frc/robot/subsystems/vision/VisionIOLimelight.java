package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.Util.LimelightHelpers;
import java.util.LinkedList;
import java.util.List;

public class VisionIOLimelight implements VisionIO {

  DoubleSubscriber latencySubscriberA;
  DoubleArraySubscriber poseArraySubscriberA;

  RobotState state = RobotState.getRobotState();

  String cameraName;
  Translation3d mountingTranslation;
  Rotation3d mountingRotation;

  public VisionIOLimelight(
      String cameraName, Translation3d mountingTranslation, Rotation3d mountingRotation) {
    this.cameraName = cameraName;
    this.mountingTranslation = mountingTranslation;
    this.mountingRotation = mountingRotation;

    NetworkTable TableA = NetworkTableInstance.getDefault().getTable(cameraName);

    latencySubscriberA = TableA.getDoubleTopic("tl").subscribe(0.0);

    poseArraySubscriberA =
        TableA.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[6]);

    setCameraPosition();
  }

  @Override
  public void readInputs(VisionInputs inputs) {

    setRobotOrientation();
    inputs.cameraName = cameraName;
    inputs.cameraConnected =
        ((Timer.getFPGATimestamp() - latencySubscriberA.getLastChange()) / 1000) < 250;

    inputs.cameraHasTarget = LimelightHelpers.getTV(cameraName);

    List<PoseObservation> poseObservationsA = new LinkedList<>();
    for (var sample : poseArraySubscriberA.readQueue()) {
      if (sample.value.length == 0) continue;
      poseObservationsA.add(
          new PoseObservation(
              (sample.timestamp * 1e-6) - (sample.value[6] * 1e-6),
              parsePose(sample.value),
              getCameraAngularVelocity(),
              0.0,
              (int) sample.value[7],
              sample.value[9],
              PoseObservationType.MEGATAG_2));
    }

    inputs.cameraAMegatagEstimate = new PoseObservation[poseObservationsA.size()];
    for (int i = 0; i < poseObservationsA.size(); i++) {
      inputs.cameraAMegatagEstimate[i] = poseObservationsA.get(i);
    }
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
        state.getLatestPose2d().getRotation().getDegrees(),
        RadiansPerSecond.of(state.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond)
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
    return RadiansPerSecond.of(state.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond);
  }
}
