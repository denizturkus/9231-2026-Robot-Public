package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.ParentDevice;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

/**
 * The main superclass interface for intake motor (hardware) interaction IO classes.
 */
public interface IntakeIO {

	/** The inputs class for the Intake Subsystem. */
	@AutoLog
	public static class IntakeIOInputs {
		public boolean rollerMotorConnected = true;
		public double rollerAppliedVoltage = 0.0;
		public double rollerRpm = 0.0;
		public double rollerSupplyCurrentAmps = 0.0;
		public double rollerStatorCurrentAmps = 0.0;
		public double rollerTemperatureCelsius = 0.0;

		public boolean armMotorConnected = true;
		public double armAppliedVoltage = 0.0;
		public double armRpm = 0.0;
		public double armPositionDegrees = 0.0;
		public double armSupplyCurrentAmps = 0.0;
		public double armStatorCurrentAmps = 0.0;
		public double armTemperatureCelsius = 0.0;
	}

	/**
	 * Updates the given roller inputs.
	 *
	 * @param inputs The inputs.
	 */
	public default void updateInputs(IntakeIOInputs inputs) {}

	// Roller //

	/**
	 * Sets the voltage of the roller motor.
	 *
	 * @param volts The voltage.
	 */
	public default void runRollerVolts(double volts) {}

	/**
	 * Stops the roller motor.
	 */
	public default void stopRollers() {
		runRollerVolts(0.0);
	}

	// Arm //

	/**
	 * Sets the voltage of the arm motor.
	 *
	 * @param volts The voltage.
	 */
	public default void runArmVolts(double volts) {}

	/**
	 * Moves the arm to the given angle.
	 *
	 * @param degrees The angle, in degrees.
	 */
	public default void runArmPosition(double degrees) {}

	/**
	 * Moves the arm to the given angle with the provided Motion Magic profile.
	 *
	 * @param degrees The angle, in degrees.
	 * @param cruiseVelocityDegPerSec The cruise velocity, in degrees per second.
	 * @param accelerationDegPerSecSq The acceleration, in degrees per second squared.
	 */
	public default void runArmPosition(
			double degrees, double cruiseVelocityDegPerSec, double accelerationDegPerSecSq) {
		runArmPosition(degrees);
	}

	/**
	 * Resets the arm encoders according to the calibration angle.
	 */
	public default void zeroArmEncoders() {}

	/**
	 * Stops the arm motor.
	 */
	public default void stopArm() {
		runArmVolts(0.0);
	}

	/**
	 * Returns the devices that can be used by Phoenix Orchestra.
	 */
	public default List<ParentDevice> getOrchestraDevices() {
		return List.of();
	}
}
