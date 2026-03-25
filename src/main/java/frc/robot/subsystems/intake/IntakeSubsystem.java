package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kArmCalibrationAngleDeg;
import static frc.robot.subsystems.intake.IntakeConstants.kArmClosedAngle;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMaxTemperature;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorID;
import static frc.robot.subsystems.intake.IntakeConstants.kArmOpenedAngle;
import static frc.robot.subsystems.intake.IntakeConstants.kIntakeRollerVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.kMotionMagicAccelerationDegPerSecSq;
import static frc.robot.subsystems.intake.IntakeConstants.kMotionMagicMaxVelocityDegPerSec;
import static frc.robot.subsystems.intake.IntakeConstants.kOuttakeRollerVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.kRetractMotionMagicAccelerationDegPerSecSq;
import static frc.robot.subsystems.intake.IntakeConstants.kRetractMotionMagicMaxVelocityDegPerSec;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMaxTemperature;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorID;

import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.util.Tracer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * The main intake subsystem to pick up fuel.
 */
public class IntakeSubsystem extends SubsystemBase {
	private final IntakeIO m_io;
	private IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();

	private Alert m_armMotorDisconnectedAlert = new Alert(
			"Intake arm motor has disconnected! (ID: " + kArmMotorID + ")", AlertType.kError);
	private Alert m_rollerMotorDisconnectedAlert = new Alert(
			"Intake roller motor has disconnected! (ID: " + kRollerMotorID + ")", AlertType.kError);

	private Alert m_motorArmOverheatAlert = new Alert("Intake arm motor is overheating! (ID: " + kArmMotorID + ")",
			AlertType.kWarning);
	private Alert m_motorRollerOverheatAlert = new Alert(
			"Intake roller motor is overheating! (ID: " + kRollerMotorID + ")", AlertType.kWarning);

	private boolean m_isZeroed;
	private double m_latestSetpoint = kArmCalibrationAngleDeg;

	private static final double kArmDefaultSetpointToleranceDeg = 5;
	private static final double kRollerRunningVoltageThresholdVolts = 1.0;

	/**
	 * Constructs a new Intake.
	 *
	 * @param io The IO implementation to use.
	 */
	public IntakeSubsystem(IntakeIO io) {
		m_io = io;
		setName("Intake");
	}

	@Override
	public void periodic() {
		Tracer.start("IntakePeriodic");
		m_io.updateInputs(m_inputs);
		Logger.processInputs("Intake", m_inputs);

		if (DriverStation.isDisabled()) {
			stopIntakeArm();
			stopRollers();
			m_latestSetpoint = m_inputs.armPositionDegrees;
		}

		m_motorArmOverheatAlert.set(m_inputs.armTemperatureCelsius > kArmMaxTemperature);
		m_motorRollerOverheatAlert.set(m_inputs.rollerTemperatureCelsius > kRollerMaxTemperature);

		Logger.recordOutput("Intake/ArmEncodersZeroed", areEncodersZeroed());
		Logger.recordOutput("Intake/ArmNearSetpoint", isArmNearSetpoint());
		Logger.recordOutput("Intake/ArmSetpoint", m_latestSetpoint);
		Logger.recordOutput(
				"Intake/ArmPositionErrorDeg",
				MathUtil.inputModulus(m_latestSetpoint - getIntakeAngle(), -180, 180));

		m_rollerMotorDisconnectedAlert.set(!m_inputs.rollerMotorConnected);
		m_armMotorDisconnectedAlert.set(!m_inputs.armMotorConnected);
		Tracer.finish("IntakePeriodic");
	}

	/**
	 * Opens the arm of the intake.
	 */
	public void openIntake() {
		setIntakeAngle(
				kArmOpenedAngle,
				kMotionMagicMaxVelocityDegPerSec,
				kMotionMagicAccelerationDegPerSecSq);
	}

	/**
	 * Closes the arm of the intake.
	 */
	public void closeIntake() {
		setIntakeAngle(
				kArmClosedAngle,
				kRetractMotionMagicMaxVelocityDegPerSec,
				kRetractMotionMagicAccelerationDegPerSecSq);
	}

	/**
	 * Runs the rollers in intake mode.
	 */
	public void runIntakeRollers() {
		setRollerVolts(kIntakeRollerVoltage);
	}

	/**
	 * Runs the rollers in outtake/eject mode.
	 */
	public void runOuttakeRollers() {
		setRollerVolts(kOuttakeRollerVoltage);
	}

	/**
	 * Stops all intake rollers.
	 */
	public void stopRollers() {
		m_io.stopRollers();
	}

	/**
	 * Moves the intake arm to the given angle.
	 *
	 * @param degs The angle, in degrees.
	 */
	public void setIntakeAngle(double degs) {
		setIntakeAngle(degs, kMotionMagicMaxVelocityDegPerSec, kMotionMagicAccelerationDegPerSecSq);
	}

	/**
	 * Moves the intake arm to the given angle using the provided motion profile.
	 *
	 * @param degs The angle, in degrees.
	 * @param cruiseVelocityDegPerSec The cruise velocity, in degrees per second.
	 * @param accelerationDegPerSecSq The acceleration, in degrees per second squared.
	 */
	public void setIntakeAngle(double degs, double cruiseVelocityDegPerSec, double accelerationDegPerSecSq) {
		m_io.runArmPosition(
				MathUtil.inputModulus(degs, -180, 180),
				cruiseVelocityDegPerSec,
				accelerationDegPerSecSq);
		m_latestSetpoint = degs;
	}

	/**
	 * Returns the intake arm's current angle.
	 *
	 * @return The intake arm's current angle, in degrees.
	 */
	public double getIntakeAngle() { return m_inputs.armPositionDegrees; }

	/**
	 * Applies the given voltage to the roller motors.
	 *
	 * @param volts The voltage to apply.
	 */
	public void setRollerVolts(double volts) {
		m_io.runRollerVolts(volts);
	}

	/**
	 * Returns the rollers' velocity.
	 *
	 * @return The rollers' velocity, in RPM.
	 */
	public double getRollerVelocity() { return m_inputs.rollerRpm; }

	/**
	 * Returns the currently applied roller voltage.
	 *
	 * @return The applied roller voltage.
	 */
	public double getRollerAppliedVoltage() { return m_inputs.rollerAppliedVoltage; }

	/**
	 * Returns whether the intake rollers are actively being driven.
	 *
	 * @return Whether the intake rollers are running.
	 */
	public boolean areRollersRunning() {
		return Math.abs(m_inputs.rollerAppliedVoltage) > kRollerRunningVoltageThresholdVolts;
	}

	/**
	 * Stops the intake arm in place.
	 */
	public void stopIntakeArm() {
		m_io.stopArm();
	}

	/**
	 * Resets the encoders of the arm. Encoders are set to read the calibration angle.
	 */
	public void zeroEncoders() {
		stopIntakeArm();
		m_io.zeroArmEncoders();
		m_isZeroed = true;
		m_latestSetpoint = kArmCalibrationAngleDeg;
	}

	/**
	 * Returns whether the encoders have been zeroed.
	 *
	 * @return Whether the encoders have been zeroed.
	 */
	public boolean areEncodersZeroed() {
		return m_isZeroed;
	}

	/**
	 * Returns whether the intake arm is near its setpoint. Uses the default tolerance.
	 *
	 * @return Whether the intake arm is near its setpoint.
	 */
	public boolean isArmNearSetpoint() {
		return isArmNearAngle(m_latestSetpoint, kArmDefaultSetpointToleranceDeg);
	}

	/**
	 * Returns whether the intake arm is near the provided angle using the default tolerance.
	 *
	 * @param targetAngleDeg The angle to compare against, in degrees.
	 * @return Whether the arm is near that angle.
	 */
	public boolean isArmNearAngle(double targetAngleDeg) {
		return isArmNearAngle(targetAngleDeg, kArmDefaultSetpointToleranceDeg);
	}

	/**
	 * Returns whether the intake arm is near the provided angle.
	 *
	 * @param targetAngleDeg The angle to compare against, in degrees.
	 * @param toleranceDeg The allowed error, in degrees.
	 * @return Whether the arm is near that angle.
	 */
	public boolean isArmNearAngle(double targetAngleDeg, double toleranceDeg) {
		return Math.abs(MathUtil.inputModulus(targetAngleDeg - getIntakeAngle(), -180, 180)) <= toleranceDeg;
	}

	/**
	 * Returns the latest setpoint of the intake arm.
	 *
	 * @return The latest setpoint of the intake arm, in degrees.
	 */
	public double getLatestArmSetpoint() { return m_latestSetpoint; }

	/**
	 * Returns the intake motors available to Phoenix Orchestra.
	 */
	public List<ParentDevice> getOrchestraDevices() {
		return m_io.getOrchestraDevices();
	}
}
