package frc.robot.Commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {

  public DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), OIConstants.kDeadband);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public static Command swerveDriveJoystick(
      Drive m_Drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), OIConstants.kDeadband);

          omega = Math.copySign(omega * omega, omega);

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * DriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
                  linearVelocity.getY() * DriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
                  omega * SwerveConstants.kAngularSpeedAt12Volts.in(RadiansPerSecond));

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          m_Drive.setSwerveSpeed(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? m_Drive.getRotation().plus(new Rotation2d(Math.PI))
                      : m_Drive.getRotation()));
        },
        m_Drive);
  }

  public static Command swerveDriveJoystickAtAngle(
      Drive m_Drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Angle> thetaSupplier) {

    ProfiledPIDController thetaController =
        new ProfiledPIDController(1, 0, 0.152, new TrapezoidProfile.Constraints(5, 10));

    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega =
              thetaController.calculate(
                  m_Drive.getRotation().getRadians(), thetaSupplier.get().in(Radians));

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * DriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
                  linearVelocity.getY() * DriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
                  omega * SwerveConstants.kAngularSpeedAt12Volts.in(RadiansPerSecond));

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          m_Drive.setSwerveSpeed(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? m_Drive.getRotation().plus(new Rotation2d(Math.PI))
                      : m_Drive.getRotation()));
        },
        m_Drive);
  }

  public static Command stopWithX(Drive m_Drive) {
    return Commands.run(m_Drive::stopX, m_Drive);
  }

  public static Command driveThroughPath(PathPlannerPath path) {
    PathConstraints constraints =
        new PathConstraints(
            SwerveConstants.PathfindingConstraints.maxVelocity,
            SwerveConstants.PathfindingConstraints.maxAcceleration,
            SwerveConstants.PathfindingConstraints.maxAngularVelocity,
            SwerveConstants.PathfindingConstraints.maxAngularAcceleration);
    return AutoBuilder.pathfindThenFollowPath(path, constraints);
  }

  public static Command driveToPose(Pose2d waypoint, double goalEndVelocity) {
    PathConstraints constraints =
        new PathConstraints(
            SwerveConstants.PathfindingConstraints.maxVelocity,
            SwerveConstants.PathfindingConstraints.maxAcceleration,
            SwerveConstants.PathfindingConstraints.maxAngularVelocity,
            SwerveConstants.PathfindingConstraints.maxAngularAcceleration);
    return AutoBuilder.pathfindToPose(waypoint, constraints);
  }
}
