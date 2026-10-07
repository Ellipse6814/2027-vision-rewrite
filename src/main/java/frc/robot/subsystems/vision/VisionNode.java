package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

public class VisionNode {
  private static List<VisionIO> IOs = new ArrayList();

  private static List<PoseObservation> rejectedPoses = new LinkedList<>();

  public static void addCamera(VisionIO cameraIO)
  {
    IOs.add(cameraIO);
  }

  private static AprilTagInputsAutoLogged apriltagInputs = new AprilTagInputsAutoLogged();
  public static void periodic() {
    for(var io : IOs)
    {
        // For future generations: if you'd like to implement object detection,
        // you'd probably want to make a new inputs class for them. So you'd have
        // apriltagInputs and objectDetectionInputs
        io.readInputs(apriltagInputs);
        Logger.processInputs("Vision/" + io.getCameraName(), apriltagInputs);

        for(var observation : apriltagInputs.cameraAMegatagEstimate)
        {
          if (observation == null) continue;
          if (!apriltagInputs.cameraHasTarget) continue;
          if(!VisionFilters.inField(observation)) continue;
          if(!VisionFilters.distanceCheck(observation)) continue;
          if(!VisionFilters.omegaAcceptable(observation)) continue;
          
          RobotState.addVisionMeasurements(observation);
        }
    }
  }

  public static void addRejectedPose(PoseObservation rejected)
  {
    rejectedPoses.add(rejected);
  }
}
