package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

/**
 * The main superclass interface for turret motor (hardware) interaction IO classes.
 */
public interface TurretIO {
	/** The inputs class for the Turret Subsystem. */
	@AutoLog
	public static class TurretIOInputs {
		public boolean motorConnected = true;
		public double appliedVoltage = 0.0;
		public double rpm = 0.0;
		public double positionDegrees = 0.0;
		public double supplyCurrentAmps = 0.0;
		public double statorCurrentAmps = 0.0;
		public double temperatureCelsius = 0.0;
	}

	/**
	 * Updates the given turret inputs.
	 *
	 * @param inputs The inputs.
	 */
	public default void updateInputs(TurretIOInputs inputs) {}

	/**
	 * Sets the voltage of the motors.
	 *
	 * @param volts The voltage.
	 */
	public default void runVolts(double volts) {}

	/**
	 * Sets the angle of the turret.
	 *
	 * @param degs The target position, in degrees.
	 */
	public default void runPosition(double degs) {}

	/**
	 * Stops the turret by setting the voltage across the motors to 0.
	 */
	public default void stop() {
		runVolts(0);
	}

	/**
	 * Zeroes the motor encoders to read the calibration angle.
	 */
	public default void zeroEncoders() {}
}
