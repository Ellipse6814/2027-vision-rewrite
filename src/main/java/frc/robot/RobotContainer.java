// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.DriveCommands;
import frc.robot.Commands.SwerveCharacterization;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final Drive m_Drive;
  private static SwerveDriveSimulation driveSimulation = null;
  private final CommandXboxController controller = new CommandXboxController(0);

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_Drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(DriveConstants.FrontLeft),
                new ModuleIOTalonFXReal(DriveConstants.FrontRight),
                new ModuleIOTalonFXReal(DriveConstants.BackLeft),
                new ModuleIOTalonFXReal(DriveConstants.BackRight));
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        driveSimulation =
            new SwerveDriveSimulation(
                SwerveConstants.driveSimConfig, new Pose2d(15, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        m_Drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(DriveConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(DriveConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(DriveConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(DriveConstants.BackRight, driveSimulation.getModules()[3]));
        resetSimulationField();

        break;

      default:
        // Replayed robot, disable IO implementations
        m_Drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_Drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_Drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_Drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_Drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive Wheel Radius Charechterization",
        SwerveCharacterization.wheelRadiusCharacterization(m_Drive));

    configureBindings();
    HttpCamera intakeCamera = new HttpCamera("Intake Camera", "http://limelight-main.local:5800");

    CameraServer.addCamera(intakeCamera);
  }

  private void configureBindings() {
    m_Drive.setDefaultCommand(
        DriveCommands.swerveDriveJoystick(
            m_Drive,
            () -> controller.getLeftX(),
            () -> controller.getLeftY(),
            () -> controller.getRightX()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
    // return Commands.none();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Pose2d initPose = new Pose2d(1, 1, new Rotation2d());
    driveSimulation.setSimulationWorldPose(initPose);
    // m_Drive.setPose(initPose);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "RobotState/Pose Error",
        driveSimulation.getSimulatedDriveTrainPose().minus(RobotState.getLatestPose2d()));
  }

  public static SwerveDriveSimulation getDriveSim() {
    return driveSimulation;
  }
}
