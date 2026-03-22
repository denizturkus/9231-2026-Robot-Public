package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kArmCalibrationAngleDeg;
import static frc.robot.subsystems.intake.IntakeConstants.kArmClosedAngle;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMaxTemperature;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorID;
import static frc.robot.subsystems.intake.IntakeConstants.kArmOpenedAngle;
import static frc.robot.subsystems.intake.IntakeConstants.kIntakeRollerVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.kOuttakeRollerVoltage;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMaxTemperature;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorID;

import frc.robot.util.Tracer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
		setIntakeAngle(kArmOpenedAngle);
	}

	/**
	 * Closes the arm of the intake.
	 */
	public void closeIntake() {
		setIntakeAngle(kArmClosedAngle);
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
		m_io.runArmPosition(MathUtil.inputModulus(degs, -180, 180));
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
		return MathUtil.isNear(m_latestSetpoint, getIntakeAngle(), kArmDefaultSetpointToleranceDeg);
	}

	/**
	 * Returns the latest setpoint of the intake arm.
	 *
	 * @return The latest setpoint of the intake arm, in degrees.
	 */
	public double getLatestArmSetpoint() { return m_latestSetpoint; }
}
