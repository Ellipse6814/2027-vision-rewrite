package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.Util.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class ModuleIOTalonFXSim extends ModuleIOTalonFX {

  private final SwerveModuleSimulation moduleSim;

  @SuppressWarnings("unchecked")
  public ModuleIOTalonFXSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      SwerveModuleSimulation moduleSim) {
    super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

    this.moduleSim = moduleSim;

    moduleSim.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(m_Drive));
    moduleSim.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(m_Turn, m_AbsouluteEncoder));
  }

  @Override
  public void readInputs(ModuleInputs inputs) {
    super.readInputs(inputs);

    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

    inputs.odometryDrivePositionsRad =
        Arrays.stream(moduleSim.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();

    inputs.odometryTurnPositions = moduleSim.getCachedSteerAbsolutePositions();
  }
}
