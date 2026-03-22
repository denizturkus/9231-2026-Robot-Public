package frc.robot.subsystems.indexer.feeder;

import static frc.robot.subsystems.indexer.feeder.FeederConstants.kFeedVoltage;
import static frc.robot.subsystems.indexer.feeder.FeederConstants.kGearboxReduction;
import static frc.robot.subsystems.indexer.feeder.FeederConstants.kUnfeedVoltage;

import frc.robot.util.Tracer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * The main feeder subsystem to carry fuel into the shooter mechanism from the hopper.
 */
public class FeederSubsystem extends SubsystemBase {
	private final String m_id = "Feeder";
	private final FeederIO m_io;
	protected final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();

	private final Alert m_disconnectedAlert;
	private FeederGoal m_currentGoal = new FeederGoal(0.0);

	/** The setpoint for the feeder motor voltage. */
	public static class FeederGoal {
		private final double m_voltage;

		/**
		 * Constructs a new FeederGoal.
		 *
		 * @param voltage The voltage to set the feeder motor to.
		 */
		public FeederGoal(double voltage) {
			m_voltage = voltage;
		}

		public double voltage() {
			return m_voltage;
		}
	}

	/**
	 * Constructs a new Feeder.
	 *
	 * @param io The IO implementation to use.
	 */
	public FeederSubsystem(FeederIO io) {
		m_io = io;
		m_disconnectedAlert = new Alert(m_id + " motor has disconnected!", AlertType.kError);
		setName(m_id);
	}

	@Override
	public void periodic() {
		Tracer.start(m_id + "_Periodic");

		if (DriverStation.isDisabled()) {
			m_currentGoal = new FeederGoal(0.0);
		}

		m_io.updateInputs(m_inputs);
		Logger.processInputs(m_id + "/Inputs", m_inputs);

		m_disconnectedAlert.set(!m_inputs.motorConnected);

		m_io.runVolts(m_currentGoal.voltage());
		Logger.recordOutput(m_id + "/VoltageSetpoint", m_currentGoal.voltage());

		Tracer.finish(m_id + "_Periodic");
	}

	/**
	 * Starts feeding balls into the shooter. Can be stopped via stop().
	 */
	public void feed() {
		m_currentGoal = new FeederGoal(kFeedVoltage);
	}

	/**
	 * Stops feeding balls into the shooter.
	 */
	public void stop() {
		m_currentGoal = new FeederGoal(0.0);
	}

	/**
	 * Moves any balls in the shooter back to the hopper by unfeeding.
	 */
	public void unfeed() {
		m_currentGoal = new FeederGoal(kUnfeedVoltage);
	}

	public void setVoltage(double volts) {
		m_currentGoal = new FeederGoal(volts);
	}

	/**
	 * Returns the current feeder goal.
	 */
	public FeederGoal getVoltageSetpoint() {
		return m_currentGoal;
	}

	/**
	 * Returns the feeder inputs.
	 */
	public FeederIOInputsAutoLogged getInputs() {
		return m_inputs;
	}

	/**
	 * Returns the angular velocity of the feeder.
	 *
	 * @return The angular velocity of the feeder, in RPM.
	 */
	public double getVelocity() {
		return m_inputs.rpm / kGearboxReduction;
	}
}