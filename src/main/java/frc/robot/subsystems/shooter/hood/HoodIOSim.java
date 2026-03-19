package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.kCenterOfMassDistance;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kDSim;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kGSim;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kGearbox;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kHoodCalibrationAngle;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMaxAngleDeg;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMinAngleDeg;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMomentOfInertia;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotionMagicAccelerationDegPerSecSq;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotionMagicMaxVelocityDegPerSec;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotorSupplyLimitAmps;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kPSim;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kSSim;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kVSim;

import frc.robot.util.sim.CustomSingleJointedArmSim;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

/**
 * The IO sim implementation for hood hardware interacions without any hardware.
 */
public class HoodIOSim implements HoodIO {
	private CustomSingleJointedArmSim m_sim;

	private boolean m_isSetpoint = false;

	private ArmFeedforward m_feedforward = new ArmFeedforward(kSSim, kGSim, kVSim * 180 / Math.PI);
	private ProfiledPIDController m_profiledController = new ProfiledPIDController(kPSim, 0, kDSim,
			new TrapezoidProfile.Constraints(kMotionMagicMaxVelocityDegPerSec, kMotionMagicAccelerationDegPerSecSq));

	/**
	 * Constructs a new HoodIOSim.
	 */
	public HoodIOSim() {
		m_sim = new CustomSingleJointedArmSim(kGearbox, kGearboxReduction, kMomentOfInertia,
				2.0 * kCenterOfMassDistance, Math.toRadians(kMinAngleDeg), Math.toRadians(kMaxAngleDeg), true,
				Math.toRadians(kHoodCalibrationAngle), 1);
		m_sim.configureCurrentLimits(kMotorSupplyLimitAmps, kMotorStatorLimitAmps);

		m_profiledController.enableContinuousInput(-180, 180);
		m_profiledController.reset(kHoodCalibrationAngle);

		// Register sim
		SimulatedArena.getInstance().addCustomSimulation(m_sim);
	}

	@Override
	public void updateInputs(HoodIOInputs inputs) {
		if (DriverStation.isDisabled()) { m_isSetpoint = false; stop(); }
		if (m_isSetpoint) {
			double pidOutput = m_profiledController.calculate(Math.toDegrees(m_sim.getAngleRads()));
			double ffOutput = m_feedforward.calculate(m_sim.getAngleRads(), m_profiledController.getGoal().velocity);

			m_sim.setInputVoltage(ffOutput + pidOutput);
		}

		Logger.recordOutput("Hood/SimPIDSetpoint", m_profiledController.getSetpoint());
		Logger.recordOutput("Hood/SimPIDGoal", m_profiledController.getGoal());
		Logger.recordOutput("Hood/SimIsSetpoint", m_isSetpoint);

		inputs.motorConnected = true;
		inputs.appliedVoltage = m_sim.getInput(0);
		inputs.rpm = Units.radiansPerSecondToRotationsPerMinute(m_sim.getVelocityRadPerSec());
		inputs.positionDegrees = Math.toDegrees(m_sim.getAngleRads());
		inputs.supplyCurrentAmps = Math.abs(m_sim.getCurrentDrawAmps() * m_sim.getInput(0) / 12.0);
		inputs.statorCurrentAmps = Math.abs(m_sim.getCurrentDrawAmps());
		inputs.temperatureCelsius = 20;
	}

	@Override
	public void runVolts(double volts) {
		m_sim.setInputVoltage(volts);
		m_isSetpoint = false;
		// m_profiledController.reset(Math.toDegrees(m_sim.getAngleRads()));
	}

	@Override
	public void runPosition(double degs) {
		m_profiledController.setGoal(degs);
		m_isSetpoint = true;
	}

	@Override
	public void zeroEncoders() {
		m_sim.setState(Math.toRadians(kHoodCalibrationAngle), 0);
		m_profiledController.reset(kHoodCalibrationAngle, 0);
		m_profiledController.setGoal(kHoodCalibrationAngle);
	}
}
