package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kDSim;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kGearbox;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMomentOfInertia;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMotorSupplyLimitAmps;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kPSim;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kSSim;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kVSim;

import frc.robot.util.sim.CustomDCMotorSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import org.ironmaple.simulation.SimulatedArena;

/** Sim IO implementation for the flywheel subsystem. */
public class FlywheelIOSim implements FlywheelIO {
	private final CustomDCMotorSim m_sim;

	// Control objects
	private final PIDController m_pid = new PIDController(kPSim, 0.0, kDSim);
	private final SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(kSSim, kVSim);

	private double m_closedLoopSetpoint = 0.0;
	private boolean m_isClosedLoop = false;

	public FlywheelIOSim() {
		m_sim = new CustomDCMotorSim(
				LinearSystemId.createDCMotorSystem(kGearbox, kMomentOfInertia, kGearboxReduction),
				kGearbox,
				2
		);

		m_sim.setSupplyCurrentLimitAmps(kMotorSupplyLimitAmps);
		m_sim.setStatorCurrentLimitAmps(kMotorStatorLimitAmps);

		SimulatedArena.getInstance().addCustomSimulation(m_sim);
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		if (m_isClosedLoop) {
			double pidOutput = m_pid.calculate(m_sim.getOutputAngularVelocityRPM(), m_closedLoopSetpoint);
			double ffOutput = m_ff.calculate(m_closedLoopSetpoint);
			m_sim.setInputVoltage(pidOutput + ffOutput);
		}

		inputs.motorConnected = true;
		inputs.appliedVoltage = m_sim.getAppliedInputVoltage();
		inputs.rpm = m_sim.getOutputAngularVelocityRPM();
		inputs.positionRevs = m_sim.getOutputAngularPositionRotations();
		inputs.supplyCurrentAmps = Math.abs(m_sim.getSupplyCurrentDrawAmps());
		inputs.statorCurrentAmps = Math.abs(m_sim.getStatorCurrentDrawAmps());
		inputs.temperatureCelsius = 20.0;
	}

	@Override
	public void runVolts(double volts) {
		m_sim.setInputVoltage(volts);
		m_closedLoopSetpoint = 0.0;
		m_isClosedLoop = false;
		m_pid.reset();
	}

	@Override
	public void runVelocity(double rpm) {
		m_closedLoopSetpoint = rpm;
		m_isClosedLoop = true;
	}

	@Override
	public void zeroEncoders() {
		m_sim.setState(0.0, 0.0);
		stop();
	}

	@Override
	public void stop() {
		runVolts(0.0);
	}
}