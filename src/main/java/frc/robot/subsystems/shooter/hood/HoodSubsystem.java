package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kHoodCalibrationAngle;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMaxAngleDeg;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMaxTemperature;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMinAngleDeg;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotorID;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kSysIdTimeout;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kSysIdVoltageRampRate;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kSysIdVoltageStep;

import frc.robot.util.Tracer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The main shooter hood subsystem to adjust shot pitch angles.
 */
public class HoodSubsystem extends SubsystemBase {
	private final HoodIO m_io;
	private HoodIOInputsAutoLogged m_inputs = new HoodIOInputsAutoLogged();

	private Alert m_motorDisconnectedAlert = new Alert("Hood motor has disconnected! (ID: " + kMotorID + ")",
			AlertType.kError);
	private Alert m_motorOverheatAlert = new Alert("Hood motor is overheating! (ID: " + kMotorID + ")",
			AlertType.kWarning);

	private boolean m_isZeroed = false;
	private double m_setpoint = kHoodCalibrationAngle;

	private SysIdRoutine m_routine;

	private static final double kAngleDefaultToleranceDeg = 4.0;

	/**
	 * Constructs a new Hood.
	 *
	 * @param io The IO implementation to use.
	 */
	public HoodSubsystem(HoodIO io) {
		m_io = io;

		// SysID routine
		m_routine = new SysIdRoutine(
				new SysIdRoutine.Config(Volts.of(kSysIdVoltageRampRate).per(Seconds.of(1).unit()),
						Volts.of(kSysIdVoltageStep), Seconds.of(kSysIdTimeout),
						(state) -> { Logger.recordOutput("Hood/SysIdState", state.toString()); }),
				new SysIdRoutine.Mechanism((voltage) -> m_io.runVolts(voltage.in(Volts)), null, this));
		setName("Hood");
	}

	@Override
	public void periodic() {
		Tracer.start("HoodPeriodic");
		m_io.updateInputs(m_inputs);
		Logger.processInputs("Hood", m_inputs);

		if (DriverStation.isDisabled()) { m_io.stop(); m_setpoint = m_inputs.positionDegrees; }

		m_motorOverheatAlert.set(m_inputs.temperatureCelsius > kMaxTemperature);

		Logger.recordOutput("Hood/EncodersZeroed", areEncodersZeroed());
		Logger.recordOutput("Hood/NearSetpoint", isNearSetpoint());
		Logger.recordOutput("Hood/Setpoint", m_setpoint);

		m_motorDisconnectedAlert.set(!m_inputs.motorConnected);
		Tracer.finish("HoodPeriodic");
	}

	/**
	 * Moves the hood to the given angle, in terms of the motor's position.
	 *
	 * @param angleDeg The new hood angle, in degrees.
	 */
	public void setHoodAngle(double angleDeg) {
		angleDeg = MathUtil.inputModulus(angleDeg, -180, 180);
		if (angleDeg > kMaxAngleDeg || angleDeg < kMinAngleDeg) {
			DriverStation.reportWarning("WARNING: Hood::setHoodAngle, angle exceeds bounds (" + angleDeg + ")", true);
			stop();
			return;
		}
		m_setpoint = angleDeg;
		m_io.runPosition(angleDeg);
	}

	/**
	 * Stops the hood.
	 */
	public void stop() {
		m_io.stop();
	}

	/**
	 * Returns the hood's current angle.
	 *
	 * @return The hood's current angle, in degrees.
	 */
	public double getAngle() { return m_inputs.positionDegrees; }

	/**
	 * Returns whether the hood is near its setpoint. Uses the default tolerance.
	 *
	 * @return Whether the hood is near its setpoint.
	 */
	public boolean isNearSetpoint() {
		return MathUtil.isNear(0,
				MathUtil.inputModulus(
						Rotation2d.fromDegrees(m_setpoint).minus(Rotation2d.fromDegrees(getAngle())).getDegrees(), -180,
						180),
				kAngleDefaultToleranceDeg);
	}

	/**
	 * Returns the latest setpoint.
	 *
	 * @return The latest setpoint, in degrees.
	 */
	public double getLatestSetpoint() { return m_setpoint; }

	/**
	 * Resets the encoders of the hood. Encoders are set to read the calibration angle.
	 */
	public void zeroEncoders() {
		stop();
		m_io.zeroEncoders();
		m_isZeroed = true;
		m_setpoint = kHoodCalibrationAngle;
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
	 * Runs a quasistatic (negligble acceleration) system identification routine.
	 *
	 * @param direction The direction.
	 * @return The command to run the identification test.
	 */
	public Command sysIdQuasistatic(Direction direction) {
		return m_routine.quasistatic(direction);
	}

	/**
	 * Runs a dynamic (non-negligble acceleration) system identification routine.
	 *
	 * @param direction The direction.
	 * @return The command to run the identification test.
	 */
	public Command sysIdDynamic(Direction direction) {
		return m_routine.dynamic(direction);
	}
}
