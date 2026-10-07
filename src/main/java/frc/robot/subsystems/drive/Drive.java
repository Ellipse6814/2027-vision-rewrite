package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  static final Lock odometryLock = new ReentrantLock();

  private final GyroIO GyroIO;
  private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysID;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(SwerveConstants.moduleTranslations);

  private final SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  // private final SwerveSetpointGenerator setpointGenerator;
  // private SwerveSetpoint swerveSetpoint;

  public Drive(
      GyroIO gyroIO,
      ModuleIO frontLeftModule,
      ModuleIO frontRightModule,
      ModuleIO backLeftModule,
      ModuleIO backRightModule) {

    this.GyroIO = gyroIO;

    modules[0] = new Module(frontLeftModule, 0, DriveConstants.FrontLeft);
    modules[1] = new Module(frontRightModule, 1, DriveConstants.FrontRight);
    modules[2] = new Module(backLeftModule, 2, DriveConstants.BackLeft);
    modules[3] = new Module(backRightModule, 3, DriveConstants.BackRight);

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    PhoenixOdometryThread.getInstance().start();

    AutoBuilder.configure(
        RobotState::getLatestPose2d,
        this::setPose,
        RobotState::getRobotRelativeChassisSpeeds,
        (speeds, feedforwards) -> setSwerveSpeed(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        SwerveConstants.kRobotConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStar());

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    sysID =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Seconds).of(5),
                Volts.of(30),
                Seconds.of(10),
                (state) -> Logger.recordOutput("Drive/SysID", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    // setpointGenerator =
    //     new SwerveSetpointGenerator(SwerveConstants.kRobotConfig, Units.rotationsToRadians(10));
    // swerveSetpoint =
    //     new SwerveSetpoint(getChassisSpeeds(), getSwerveStates(), DriveFeedforwards.zeros(4));
  }

  @Override
  public void periodic() {
    odometryLock.lock();

    GyroIO.readInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    odometryLock.unlock();

    double[] sampleTimestamps = modules[0].getOdometryTimestamps();

    RobotState.addDriveMeasurements(
        sampleTimestamps,
        getSwervePositions(),
        gyroInputs.odometryYawPositions,
        getChassisSpeedsFieldRelative(),
        getChassisSpeeds(),
        gyroInputs.gyroRoll,
        gyroInputs.gyroPitch,
        gyroInputs.gyroPitchVelocity,
        gyroInputs.gyroRollVelocity,
        gyroInputs.robotXAceleration,
        gyroInputs.robotYAceleration);
  }

  public void setSwerveSpeed(ChassisSpeeds speeds) {
    // swerveSetpoint = setpointGenerator.generateSetpoint(swerveSetpoint, speeds, 0.02);
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setPoints = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setPoints, DriveConstants.kSpeedAt12Volts);

    Logger.recordOutput("SwerveStates/Setpoints", setPoints);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    for (var module : modules) {
      module.setDesiredState(setPoints[module.getIndex()]);
    }

    Logger.recordOutput("SwerveStates/OptimizedSetpoint", setPoints);
  }

  public void stop() {
    setSwerveSpeed(new ChassisSpeeds());
  }

  public void stopX() {
    Rotation2d[] headings = new Rotation2d[4];

    for (int i = 0; i < 4; i++) {
      headings[i] = SwerveConstants.moduleTranslations[i].getAngle();
    }

    kinematics.resetHeadings(headings);
    stop();
  }

  public void runCharacterization(double output) {
    for (var module : modules) {
      module.runCharacterization(output);
    }
  }

  public SwerveModulePosition[] getSwervePositions() {
    SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];

    for (var module : modules) {
      swervePositions[module.getIndex()] = module.getSwervePosition();
    }

    return swervePositions;
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getSwerveStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (var module : modules) {
      states[module.getIndex()] = module.getState();
    }

    return states;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveStates());
  }

  public ChassisSpeeds getChassisSpeedsFieldRelative() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
  }

  public Rotation2d[] getModuleAngles() {
    Rotation2d[] output = new Rotation2d[4];

    for (var module : modules) {
      output[module.getIndex()] = module.getAngle();
    }
    return output;
  }

  public Rotation2d[] getModuleDriveAngles() {
    Rotation2d[] output = new Rotation2d[4];

    for (var module : modules) {
      output[module.getIndex()] = module.getDriveAngle();
    }
    return output;
  }

  public AngularVelocity getAvgModuleOmega() {
    AngularVelocity omega = RadiansPerSecond.of(0);

    for (var module : modules) {
      omega.plus(module.getOmega().div(4.0));
    }
    return omega;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return new Rotation2d(gyroInputs.gyroYaw);
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    if (Constants.currentMode == Constants.Mode.SIM) {
      RobotContainer.getDriveSim().setSimulationWorldPose(pose);
    }
    pose = new Pose2d(pose.getTranslation(), getRotation());
    RobotState.resetPose(pose, getSwervePositions());
  }

  public void resetGyro() {
    GyroIO.resetGyro();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysID.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysID.dynamic(direction));
  }
}
