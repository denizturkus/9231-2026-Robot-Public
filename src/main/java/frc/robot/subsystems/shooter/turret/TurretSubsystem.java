package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kDeadzoneLookupRange;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMaxAngleDeg;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMaxTemperature;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMinAngleDeg;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotorID;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kSysIdTimeout;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kSysIdVoltageRampRate;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kSysIdVoltageStep;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kTurretCalibrationAngle;

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
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The main shooter turret subsystem to adjust shot yaw angles.
 */
public class TurretSubsystem extends SubsystemBase {
	private final TurretIO m_io;
	private TurretIOInputsAutoLogged m_inputs = new TurretIOInputsAutoLogged();

	private Alert m_motorDisconnectedAlert = new Alert("Turret motor has disconnected! (ID: " + kMotorID + ")",
			AlertType.kError);
	private Alert m_motorOverheatAlert = new Alert("Turret motor is overheating! (ID: " + kMotorID + ")",
			AlertType.kWarning);

	private double m_latestSetpoint;
	private boolean m_isZeroed = false;

	private SysIdRoutine m_routine;

	private static final double kAngleDefaultToleranceDeg = 5;

	/**
	 * Constructs a new Turret.
	 *
	 * @param io The IO implementation to use.
	 */
	public TurretSubsystem(TurretIO io) {
		m_io = io;

		// SysID routine
		m_routine = new SysIdRoutine(
				new SysIdRoutine.Config(Volts.of(kSysIdVoltageRampRate).per(Seconds.of(1).unit()),
						Volts.of(kSysIdVoltageStep), Seconds.of(kSysIdTimeout),
						(state) -> { Logger.recordOutput("Turret/SysIdState", state.toString()); }),
				new SysIdRoutine.Mechanism((voltage) -> m_io.runVolts(voltage.in(Volts)), null, this));
		setName("Turret");
	}

	@Override
	public void periodic() {
		Tracer.start("TurretPeriodic");
		m_io.updateInputs(m_inputs);
		Logger.processInputs("Turret", m_inputs);

		if (DriverStation.isDisabled()) { m_latestSetpoint = m_inputs.positionDegrees; stop(); }

		m_motorOverheatAlert.set(m_inputs.temperatureCelsius > kMaxTemperature);

		Logger.recordOutput("Turret/EncodersZeroed", areEncodersZeroed());
		Logger.recordOutput("Turret/NearSetpoint", isNearSetpoint());
		Logger.recordOutput("Turret/Setpoint", m_latestSetpoint);

		m_motorDisconnectedAlert.set(!m_inputs.motorConnected);
		Tracer.finish("TurretPeriodic");
	}

	/**
	 * Moves the turret to the given angle.
	 *
	 * @param angleDeg The new turret angle, in degrees.
	 */
	public void setTurretAngle(double angleDeg) {
		angleDeg = MathUtil.inputModulus(angleDeg, -180, 180);

		boolean foundBest = false;
		double optimizedAngle = 0;
		for (int i = -kDeadzoneLookupRange; i <= kDeadzoneLookupRange; i++) {
			double potentialSetpoint = angleDeg + 360 * i;
			if (potentialSetpoint < kMinAngleDeg || potentialSetpoint > kMaxAngleDeg) {
				continue;
			} else {
				if (!foundBest) { optimizedAngle = potentialSetpoint; foundBest = true; }
				if (Math.abs(m_latestSetpoint - potentialSetpoint) < Math.abs(m_latestSetpoint - optimizedAngle)) {
					optimizedAngle = potentialSetpoint;
				}
			}
		}

		m_latestSetpoint = optimizedAngle;
		m_io.runPosition(optimizedAngle);
	}

	/**
	 * Sets the voltage of the motors.
	 *
	 * @param volts The voltage.
	 */
	public void setVoltage(double volts) {
		m_io.runVolts(volts);
	}

	/**
	 * Stops the turret.
	 */
	public void stop() {
		m_io.stop();
	}

	/**
	 * Returns the turret's current angle.
	 *
	 * @return The turret's current angle, in degrees.
	 */
	public double getAngle() { return MathUtil.inputModulus(m_inputs.positionDegrees, -180, 180); }

	/**
	 * Returns whether the turret is near its setpoint. Uses the default tolerance.
	 *
	 * @return Whether the turret is near its setpoint.
	 */
	public boolean isNearSetpoint() {
		return MathUtil.isNear(0,
				Rotation2d.fromDegrees(m_latestSetpoint).minus(Rotation2d.fromDegrees(getAngle())).getDegrees(),
				kAngleDefaultToleranceDeg);
	}

	/**
	 * Returns the latest setpoint.
	 *
	 * @return The latest setpoint, in degrees.
	 */
	public double getLatestSetpoint() { return m_latestSetpoint; }

	/**
	 * Resets the encoders of the turret. Encoders are set to read the calibration angle.
	 */
	public void zeroEncoders() {
		stop();
		m_io.zeroEncoders();
		m_isZeroed = true;
		m_latestSetpoint = kTurretCalibrationAngle;
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
