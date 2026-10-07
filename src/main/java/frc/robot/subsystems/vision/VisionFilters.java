package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.RobotState;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

public class VisionFilters {
  public static boolean inField(PoseObservation observation) {
    if (observation.pose().getX() < 0
        || observation.pose().getX() > FieldConstants.kFieldMaxX.in(Meters)
        || observation.pose().getY() < 0
        || observation.pose().getY() > FieldConstants.kFieldMaxY.in(Meters)) {
      VisionNode.addRejectedPose(observation);
      return false;
    }

    return true;
  }

  public static boolean distanceCheck(PoseObservation observation) {
    if (((RobotState.getLatestPose2d().minus(observation.pose().toPose2d())).getTranslation().getNorm()
            > VisionConstants.kMaxPoseError.in(Meters))
        || observation.pose() == Pose3d.kZero) {

      VisionNode.addRejectedPose(observation);
      return false;
    }
    return true;
  }

  public static boolean omegaAcceptable(PoseObservation observation) {
    if (observation.angularVelocity().in(RadiansPerSecond)
        > VisionConstants.kMaxCameraOmega.in(RadiansPerSecond)) {
      
        VisionNode.addRejectedPose(observation);
        return false;
    }
    return true;
  }
}
