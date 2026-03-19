package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.kDSim;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kGearbox;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMomentOfInertia;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotionMagicAccelerationDegPerSecSq;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotionMagicMaxVelocityDegPerSec;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotorSupplyLimitAmps;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kPSim;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kSSim;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kTurretCalibrationAngle;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kVSim;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.sim.CustomDCMotorSim;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

/**
 * The IO sim implementation for turret hardware interacions without any hardware.
 */
public class TurretIOSim implements TurretIO {
	private CustomDCMotorSim m_sim;

	private ProfiledPIDController m_pid = new ProfiledPIDController(kPSim, 0, kDSim,
			new TrapezoidProfile.Constraints(kMotionMagicMaxVelocityDegPerSec, kMotionMagicAccelerationDegPerSecSq));
	private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(kSSim, kVSim);
	private boolean m_isSetpoint = false;

	/**
	 * Constructs a new TurretIOSim.
	 */
	public TurretIOSim() {
		m_sim = new CustomDCMotorSim(LinearSystemId.createDCMotorSystem(kGearbox, kMomentOfInertia, kGearboxReduction),
				kGearbox, 1);
		m_sim.setSupplyCurrentLimitAmps(kMotorSupplyLimitAmps);
		m_sim.setStatorCurrentLimitAmps(kMotorStatorLimitAmps);

		m_pid.reset(kTurretCalibrationAngle);

		// Register roller sim
		SimulatedArena.getInstance().addCustomSimulation(m_sim);
	}

	@Override
	public void updateInputs(TurretIOInputs inputs) {
		if (DriverStation.isDisabled()) { m_isSetpoint = false; stop(); }
		if (m_isSetpoint) {
			double pidOutput = m_pid.calculate(Math.toDegrees(m_sim.getOutputAngularPositionRad()));
			double ffOutput = m_feedforward.calculateWithVelocities(
					Math.toDegrees(m_sim.getOutputAngularVelocityRadPerSec()), m_pid.getSetpoint().velocity);
			m_sim.setInputVoltage(ffOutput + pidOutput);
		}

		Logger.recordOutput("Turret/SimPIDSetpoint", m_pid.getSetpoint());
		Logger.recordOutput("Turret/SimPIDGoal", m_pid.getGoal());
		Logger.recordOutput("Turret/SimIsSetpoint", m_isSetpoint);

		inputs.motorConnected = true;
		inputs.appliedVoltage = m_sim.getAppliedInputVoltage();
		inputs.rpm = m_sim.getOutputAngularVelocityRPM();
		inputs.positionDegrees = Math.toDegrees(m_sim.getOutputAngularPositionRad());
		inputs.supplyCurrentAmps = Math.abs(m_sim.getSupplyCurrentDrawAmps());
		inputs.statorCurrentAmps = Math.abs(m_sim.getStatorCurrentDrawAmps());
		inputs.temperatureCelsius = 20;
	}

	@Override
	public void runVolts(double volts) {
		m_sim.setInputVoltage(volts);
		m_isSetpoint = false;
		// m_pid.reset(Units.rotationsToDegrees(m_sim.getOutputAngularPositionRotations()));
	}

	@Override
	public void runPosition(double degrees) {
		m_pid.setGoal(degrees);
		m_isSetpoint = true;
	}

	@Override
	public void zeroEncoders() {
		m_sim.setState(Math.toRadians(kTurretCalibrationAngle), 0);
		m_pid.reset(kTurretCalibrationAngle, 0);
		m_pid.setGoal(kTurretCalibrationAngle);
	}
}
