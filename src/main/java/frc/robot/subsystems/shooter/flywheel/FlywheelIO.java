package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

/**
 * The main superclass interface for shooter motor (hardware) interaction IO classes.
 */
public interface FlywheelIO {
	/** The inputs class for the Flywheel Subsystem. */
	@AutoLog
	public class FlywheelIOInputs {
		public boolean motorConnected = true;
		public double appliedVoltage = 0.0;
		public double rpm = 0.0;
		public double positionRevs = 0.0;
		public double supplyCurrentAmps = 0.0;
		public double statorCurrentAmps = 0.0;
		public double temperatureCelsius = 0.0;
	}

	/**
	 * Updates the given flywheel inputs.
	 *
	 * @param inputs The inputs.
	 */
	public default void updateInputs(FlywheelIOInputs inputs) {}

	/**
	 * Sets the voltage of the motors.
	 *
	 * @param volts The voltage.
	 */
	public default void runVolts(double volts) {}

	/**
	 * Sets the velocity of the flywheel.
	 *
	 * @param rpm The target velocity in RPM.
	 */
	public default void runVelocity(double rpm) {}

	/**
	 * Zeroes the motor encoders to read 0.
	 */
	public default void zeroEncoders() {}

	/**
	 * Stops the flywheel by setting the voltage across the motors to 0.
	 */
	public default void stop() {
		runVelocity(0);
	}
}
