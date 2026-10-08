package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

public class VisionFilters {
  public static boolean inField(PoseObservation observation) {
    return !(observation.pose().getX() < 0
        || observation.pose().getX() > FieldConstants.kFieldMaxX.in(Meters)
        || observation.pose().getY() < 0
        || observation.pose().getY() > FieldConstants.kFieldMaxY.in(Meters));
  }

  public static boolean closeToCurrent(PoseObservation observation) {
    return (RobotState.getLatestPose2d().minus(observation.pose().toPose2d()))
            .getTranslation()
            .getNorm()
        < VisionConstants.kMaxPoseError.in(Meters);
  }

  public static boolean isValid(PoseObservation observation) {
    return !observation.pose().getTranslation().equals(Translation3d.kZero);
  }

  public static boolean omegaAcceptable(PoseObservation observation) {
    return Math.abs(observation.angularVelocity().in(RadiansPerSecond))
        < VisionConstants.kMaxCameraOmega.in(RadiansPerSecond);
  }
}
