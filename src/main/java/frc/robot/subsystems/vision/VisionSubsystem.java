package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {

  private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();
  private final VisionIO[] ioArray;

  List<PoseObservation> rejectedObservations = new LinkedList<>();
  List<PoseObservation> acceptedObservations = new LinkedList<>();

  RobotState state = RobotState.getRobotState();

  private final boolean enableDetailedLogging = false;

  public VisionSubsystem(VisionIO... io) {
    this.ioArray = io;
  }

  @Override
  public void periodic() {
    for (var io : ioArray) {
      io.readInputs(inputs);
      Logger.processInputs("Vision/" + io.getCameraName(), inputs);
      processObservations(inputs);
      // Logger.recordOutput(
      //     "Vision/RejectedPoses",
      //     rejectedObservations.toArray(new PoseObservation[rejectedObservations.size()]));
      // Logger.recordOutput(
      //     "Vision/AcceptedPoses",
      //     acceptedObservations.toArray(new PoseObservation[acceptedObservations.size()]));
    }
  }

  private void processObservations(VisionInputsAutoLogged inputs) {
    for (var observation : inputs.cameraAMegatagEstimate) {
      if (observation == null) continue;
      if (!inputs.cameraHasTarget) continue;
      if (!inField(observation)) continue;
      if (!poseValid(observation)) continue;
      if (!omegaAceptable(observation)) continue;
      acceptedObservations.add(observation);
      state.addVisionMeasurements(observation, getSTD(observation));
    }
  }

  private boolean inField(PoseObservation observation) {
    if (observation.pose().getX() < 0
        || observation.pose().getX() > FieldConstants.kFieldMaxX.in(Meters)) {
      rejectedObservations.add(observation);
      if (enableDetailedLogging) {
        System.out.println("rejected field");
      }
      return false;
    }
    if (observation.pose().getY() < 0
        || observation.pose().getY() > FieldConstants.kFieldMaxY.in(Meters)) {
      rejectedObservations.add(observation);
      if (enableDetailedLogging) {
        System.out.println("rejected field");
      }
      return false;
    }

    return true;
  }

  private boolean poseValid(PoseObservation observation) {
    if (((state.getLatestPose2d().minus(observation.pose().toPose2d())).getTranslation().getNorm()
            > VisionConstants.kMaxPoseError.in(Meters))
        || observation.pose() == Pose3d.kZero) {
      rejectedObservations.add(observation);
      if (enableDetailedLogging) {
        System.out.println("rejected valid");
      }
      return false;
    }
    return true;
  }

  private boolean omegaAceptable(PoseObservation observation) {
    if (observation.angularVelocity().in(RadiansPerSecond)
        > VisionConstants.kMaxCameraOmega.in(RadiansPerSecond)) {
      rejectedObservations.add(observation);
      if (enableDetailedLogging) {
        System.out.println("rejected omega");
      }
      return false;
    }
    return true;
  }

  private Vector<N3> getSTD(PoseObservation observation) {
    double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
    double linearStdDev = VisionConstants.kLinearStdDevBaseline * stdDevFactor;
    double angularStdDev = VisionConstants.kAngularStdDevBaseline * stdDevFactor;

    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }
}
