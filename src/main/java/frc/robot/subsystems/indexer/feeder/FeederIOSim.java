package frc.robot.subsystems.indexer.feeder;

import frc.robot.util.sim.CustomDCMotorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import org.ironmaple.simulation.SimulatedArena;

/** The IO sim implementation for the feeder interacions without any hardware. */
public class FeederIOSim implements FeederIO {

	public static double[] kSystemStandardDeviations = new double[] { 0.5, 0.5, 0.2 };

	private CustomDCMotorSim m_sim;

	/**
	 * Constructs a new FeederIOSim.
	 *
	 * @param gearbox             The motor gearbox. Do <b>not</b> use
	 *                            {@link DCMotor#withReduction(double)}.
	 * @param moi                 The MOI of the rollers, in kg*m^2.
	 * @param reduction           The reduction of each motor.
	 * @param numMotors           The number of motors driving the rollers.
	 * @param supplyCurrentLimit  The supply current limit per motor.
	 * @param statorCurrentLimit  The stator/torque current limit per motor.
	 * @param simulateSystemNoise Whether to add measurement standard deviations to the simulation.
	 */
	public FeederIOSim() {

		// Create sim
		m_sim = new CustomDCMotorSim(LinearSystemId.createDCMotorSystem(FeederConstants.kGearbox, FeederConstants.kMomentOfInertia, FeederConstants.kGearboxReduction), FeederConstants.kGearbox, 1,
				false ? kSystemStandardDeviations : new double[3]);

		// Register sim
		SimulatedArena.getInstance().addCustomSimulation(m_sim);
	}

	@Override
	public void updateInputs(FeederIOInputs inputs) {
		// Update output data
		inputs.motorConnected = true;
		inputs.appliedVoltage = m_sim.getAppliedInputVoltage();
		inputs.positionRevs = m_sim.getOutputAngularPositionRotations() * m_sim.getGearing();
		inputs.rpm = m_sim.getOutputAngularVelocityRPM() * m_sim.getGearing();
		inputs.statorCurrentAmps = Math.abs(m_sim.getStatorCurrentDrawAmps());
		inputs.supplyCurrentAmps = Math.abs(m_sim.getSupplyCurrentDrawAmps());
		inputs.temperatureCelsius = 20;
	}

	@Override
	public void runVolts(double volts) {
		m_sim.setInputVoltage(volts);
	}
}
