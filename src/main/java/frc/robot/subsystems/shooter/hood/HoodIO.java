package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

/**
 * The main superclass interface for hood motor (hardware) interaction IO classes.
 */
public interface HoodIO {

	// NOTE: All RPMs and positions are the output values, not motor.

	/** The inputs class for the Hood Subsystem. */
	@AutoLog
	public class HoodIOInputs {
		public boolean motorConnected = true;
		public double appliedVoltage = 0.0;
		public double rpm = 0.0;
		public double positionDegrees = 0.0;
		public double supplyCurrentAmps = 0.0;
		public double statorCurrentAmps = 0.0;
		public double temperatureCelsius = 0.0;
	}

	/**
	 * Updates the given hood inputs.
	 *
	 * @param inputs The inputs.
	 */
	public default void updateInputs(HoodIOInputs inputs) {}

	/**
	 * Sets the voltage of the motors.
	 *
	 * @param volts The voltage.
	 */
	public default void runVolts(double volts) {}

	/**
	 * Sets the position of the hood.
	 *
	 * @param degs The target position, in degrees.
	 */
	public default void runPosition(double degs) {}

	/**
	 * Stops the hood by setting the voltage across the motors to 0.
	 */
	public default void stop() {
		runVolts(0);
	}

	/**
	 * Zeroes the hood encoders to read the calibration angle.
	 */
	public default void zeroEncoders() {}
}
