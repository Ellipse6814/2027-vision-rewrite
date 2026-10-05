package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionInputs {
    public String cameraName = "";
    public boolean cameraConnected = false;
    public boolean cameraHasTarget = false;

    public PoseObservation[] cameraAMegatagEstimate = new PoseObservation[0];
  }

  record PoseObservation(
      double timestamp,
      Pose3d pose,
      AngularVelocity angularVelocity,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  public default void readInputs(VisionInputs inputs) {}

  public default void setCameraPosition() {}

  public default void setRobotOrientation() {}

  public default String getCameraName() {
    return "";
  }
}
