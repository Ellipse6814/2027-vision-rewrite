package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Util.LimelightHelpers;

public class VisionIOTurret extends VisionIOLimelight {

  Distance turretRadius;

  public VisionIOTurret(
      String cameraName,
      Translation3d mountingTranslation,
      Rotation3d mountingRotation,
      Distance turretRadius) {

    super(cameraName, mountingTranslation, mountingRotation);
    this.turretRadius = turretRadius;
  }

  @Override
  public void readInputs(VisionInputs inputs) {
    setCameraPositionTurret();
    super.readInputs(inputs);
  }

  public void setCameraPositionTurret() {
    LimelightHelpers.setCameraPose_RobotSpace(
        cameraName,
        mountingTranslation.getX()
            + ((turretRadius.in(Meters)
                * Math.cos(super.state.getTurretRobotSpaceYaw().getRadians()))),
        -(mountingTranslation.getY()
            + ((turretRadius.in(Meters)
                * Math.sin(super.state.getTurretRobotSpaceYaw().getRadians())))),
        mountingTranslation.getZ(),
        mountingRotation.getX(),
        mountingRotation.getY(),
        state.getTurretRobotSpaceYaw().getDegrees());
  }

  @Override
  protected AngularVelocity getCameraAngularVelocity() {
    return super.getCameraAngularVelocity().plus(state.getTurretFieldRelativeVelocity());
  }
}
