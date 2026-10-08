package frc.robot.subsystems.vision;

import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class VisionNode {
  private static final List<VisionIO> IOs = new ArrayList<>();

  private static final List<RejectedPoseObservation> rejectedPoses = new LinkedList<>();

  public static void addCamera(VisionIO cameraIO) {
    IOs.add(cameraIO);
  }

  private static AprilTagInputsAutoLogged apriltagInputs = new AprilTagInputsAutoLogged();

  public static void periodic() {
    for (var io : IOs) {
      // For future generations: if you'd like to implement object detection,
      // you'd probably want to make a new inputs class for them. So you'd have
      // apriltagInputs and objectDetectionInputs
      io.readInputs(apriltagInputs);
      Logger.processInputs("Vision/" + io.getCameraName(), apriltagInputs);

      for (var observation : apriltagInputs.poseObservations) {
        if (observation == null) continue;
        if (!apriltagInputs.cameraHasTarget) continue;
        if (!VisionFilters.inField(observation)) {
          addRejectedPose(observation, RejectionReason.OUTSIDE_FIELD);
          continue;
        }
        if (!VisionFilters.isValid(observation)) {
          addRejectedPose(observation, RejectionReason.INVALID_ZERO_ZERO);
          continue;
        }
        if (!VisionFilters.closeToCurrent(observation)) {
          addRejectedPose(observation, RejectionReason.TOO_FAR_FROM_CURRENT);
          continue;
        }
        if (!VisionFilters.omegaAcceptable(observation)) {
          addRejectedPose(observation, RejectionReason.OMEGA_TOO_FAST);
          continue;
        }

        RobotState.addVisionMeasurements(observation);
      }
    }
  }

  enum RejectionReason {
    OUTSIDE_FIELD,
    INVALID_ZERO_ZERO,
    TOO_FAR_FROM_CURRENT,
    OMEGA_TOO_FAST
  }

  record RejectedPoseObservation(PoseObservation poseObservation, RejectionReason reason) {}

  public static void addRejectedPose(PoseObservation rejected, RejectionReason reason) {
    rejectedPoses.add(new RejectedPoseObservation(rejected, reason));
  }
}
