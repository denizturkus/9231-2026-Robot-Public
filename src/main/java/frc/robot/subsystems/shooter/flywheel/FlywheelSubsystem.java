package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kFollowerMotorID;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kLeadMotorID;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMaxAllowedRPM;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMaxTemperature;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kSysIdTimeout;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kSysIdVoltageRampRate;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kSysIdVoltageStep;

import frc.robot.util.Tracer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Main flywheel subsystem for the shooter.
 */
public class FlywheelSubsystem extends SubsystemBase {
	private final FlywheelIO m_io;
	private final FlywheelIOInputsAutoLogged m_inputs = new FlywheelIOInputsAutoLogged();

	private double m_setpointRPM = 0.0;
	private boolean m_isZeroed = false;

	private final Alert m_motorDisconnectedAlert =
			new Alert(
					"Flywheel motor disconnected! (Lead ID: " + kLeadMotorID
							+ ", Follower ID: " + kFollowerMotorID + ")",
					AlertType.kError);

	private final Alert m_motorOverheatAlert =
			new Alert(
					"Flywheel motor overheating! (Lead ID: " + kLeadMotorID
							+ ", Follower ID: " + kFollowerMotorID + ")",
					AlertType.kWarning);

	private final SysIdRoutine m_routine;

	private static final double kVelocityDefaultToleranceRPM = 50.0;

	/**
	 * Constructs a new Flywheel.
	 *
	 * @param io The IO implementation to use.
	 */
	public FlywheelSubsystem(FlywheelIO io) {
		m_io = io;

setName("Flywheel");

m_routine = new SysIdRoutine(
		new SysIdRoutine.Config(
				Volts.of(kSysIdVoltageRampRate).per(Seconds),
				Volts.of(kSysIdVoltageStep),
				Seconds.of(kSysIdTimeout),
				null),
		new SysIdRoutine.Mechanism(
				(volts) -> m_io.runVolts(volts.in(Volts)),
				null,
				this));
    }

	@Override
	public void periodic() {
		Tracer.start("FlywheelPeriodic");

		m_io.updateInputs(m_inputs);
		Logger.processInputs("Flywheel", m_inputs);

		if (DriverStation.isDisabled()) {
			m_io.stop();
			m_setpointRPM = 0.0;
		}

		m_motorDisconnectedAlert.set(!m_inputs.motorConnected);
		m_motorOverheatAlert.set(m_inputs.temperatureCelsius > kMaxTemperature);

		Logger.recordOutput("Flywheel/SetpointRPM", m_setpointRPM);
		Logger.recordOutput("Flywheel/NearSetpoint", isNearSetpoint());
		Logger.recordOutput("Flywheel/EncodersZeroed", areEncodersZeroed());

		Tracer.finish("FlywheelPeriodic");
	}

	/**
	 * Stops the flywheel.
	 */
	public void stop() {
		m_io.stop();
		m_setpointRPM = 0.0;
	}

	/**
	 * Sets the flywheel angular velocity.
	 *
	 * @param rpm Target velocity in RPM.
	 */
	public void setVelocity(double rpm) {
		if (Math.abs(rpm) > kMaxAllowedRPM) {
			DriverStation.reportWarning(
					"WARNING: Flywheel::setVelocity, velocity exceeds limit: " + rpm,
					true);
			stop();
			return;
		}

		m_io.runVelocity(rpm);
		m_setpointRPM = rpm;
	}

	/**
	 * Runs the flywheel open-loop with voltage.
	 *
	 * @param volts Target voltage.
	 */
	public void setVoltage(double volts) {
		m_io.runVolts(volts);
	}

	/**
	 * Returns whether the flywheel is near its current setpoint.
	 *
	 * @return Whether the flywheel is near setpoint.
	 */
	public boolean isNearSetpoint() {
		return MathUtil.isNear(m_setpointRPM, m_inputs.rpm, kVelocityDefaultToleranceRPM);
	}

	/**
	 * Returns the latest velocity setpoint.
	 *
	 * @return Latest setpoint in RPM.
	 */
	public double getLatestSetpoint() {
		return m_setpointRPM;
	}

	/**
	 * Returns current flywheel velocity.
	 *
	 * @return Current velocity in RPM.
	 */
	public double getVelocity() {
		return m_inputs.rpm;
	}

	/**
	 * Zeroes the flywheel encoder reading.
	 * Mostly useful for logging/diagnostics, not normal flywheel control.
	 */
	public void zeroEncoders() {
		m_io.zeroEncoders();
		m_isZeroed = true;
		stop();
	}

	/**
	 * Returns whether encoders have been zeroed.
	 *
	 * @return Whether zeroEncoders() has been called.
	 */
	public boolean areEncodersZeroed() {
		return m_isZeroed;
	}

	/**
	 * Runs a quasistatic system identification routine.
	 *
	 * @param direction Test direction.
	 * @return SysId command.
	 */
	public Command sysIdQuasistatic(Direction direction) {
		return m_routine.quasistatic(direction);
	}

	/**
	 * Runs a dynamic system identification routine.
	 *
	 * @param direction Test direction.
	 * @return SysId command.
	 */
	public Command sysIdDynamic(Direction direction) {
		return m_routine.dynamic(direction);
	}
}