package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kAngleToleranceDeg;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kD;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kG;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kHomeAngleDeg;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMaxAngleDeg;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMaxTemperatureCelsius;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMinAngleDeg;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotionMagicAccelerationDegPerSecSq;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotionMagicCruiseVelocityDegPerSec;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotorCANBus;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotorID;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotorInverted;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotorSupplyLimitAmps;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kP;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kS;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kStatusSignalFrequencyHz;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kSysIdTimeout;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kSysIdVoltageRampRate;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kSysIdVoltageStep;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kUseFOC;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kV;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

/** Direct TalonFX hood subsystem for a single Kraken X60. */
public class HoodSubsystem extends SubsystemBase {
	private enum ControlMode {
		NEUTRAL,
		VOLTAGE,
		MOTION_MAGIC
	}

	private final boolean hardwareEnabled;
	private final TalonFX motor;

	private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(kUseFOC);
	private final MotionMagicVoltage motionMagicRequest =
			new MotionMagicVoltage(0.0).withEnableFOC(kUseFOC).withSlot(0);
	private final NeutralOut neutralRequest = new NeutralOut();

	private final StatusSignal<Voltage> motorVoltageSignal;
	private final StatusSignal<AngularVelocity> velocitySignal;
	private final StatusSignal<Angle> positionSignal;
	private final StatusSignal<Temperature> temperatureSignal;

	private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
	private final Alert motorDisconnectedAlert =
			new Alert("Hood motor disconnected! (ID: " + kMotorID + ")", AlertType.kError);
	private final Alert motorOverheatAlert =
			new Alert("Hood motor overheating! (ID: " + kMotorID + ")", AlertType.kWarning);
	private final SysIdRoutine routine;

	private double measuredPositionRotations = Units.degreesToRotations(kHomeAngleDeg);
	private double measuredVelocityRotationsPerSec = 0.0;
	private double measuredAngleDeg = kHomeAngleDeg;
	private double measuredVelocityDegPerSec = 0.0;
	private double appliedVolts = 0.0;
	private double temperatureCelsius = 0.0;

	private double requestedSetpointDeg = kHomeAngleDeg;
	private double clampedSetpointDeg = kHomeAngleDeg;
	private boolean motorConnected = false;
	private boolean encodersZeroed = false;
	private ControlMode controlMode = ControlMode.NEUTRAL;

	public HoodSubsystem() {
		setName("Hood");
		hardwareEnabled = Constants.currentMode == Constants.Mode.REAL;

		if (hardwareEnabled) {
			motor = kMotorCANBus.isEmpty() ? new TalonFX(kMotorID) : new TalonFX(kMotorID, kMotorCANBus);
			configureMotor();

			motorVoltageSignal = motor.getMotorVoltage();
			velocitySignal = motor.getVelocity();
			positionSignal = motor.getPosition();
			temperatureSignal = motor.getDeviceTemp();

			BaseStatusSignal.setUpdateFrequencyForAll(
					kStatusSignalFrequencyHz,
					motorVoltageSignal,
					velocitySignal,
					positionSignal,
					temperatureSignal);
			ParentDevice.optimizeBusUtilizationForAll(motor);
		} else {
			motor = null;
			motorVoltageSignal = null;
			velocitySignal = null;
			positionSignal = null;
			temperatureSignal = null;
			motorConnected = true;
		}

		routine = new SysIdRoutine(
				new SysIdRoutine.Config(
						Volts.of(kSysIdVoltageRampRate).per(Seconds.of(1).unit()),
						Volts.of(kSysIdVoltageStep),
						Seconds.of(kSysIdTimeout),
						(state) -> Logger.recordOutput("Hood/SysIdState", state.toString())),
				new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, this));
	}

	private void configureMotor() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;	
		config.MotorOutput.Inverted =
				kMotorInverted
						? InvertedValue.Clockwise_Positive
						: InvertedValue.CounterClockwise_Positive;

		config.CurrentLimits.SupplyCurrentLimit = kMotorSupplyLimitAmps;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = kMotorStatorLimitAmps;
		config.CurrentLimits.StatorCurrentLimitEnable = true;

		config.Feedback.SensorToMechanismRatio = kGearboxReduction;

		config.Slot0.kP = kP * 360.0;
		config.Slot0.kD = kD * 360.0;
		config.Slot0.kS = kS;
		config.Slot0.kV = kV * 360.0;
		config.Slot0.kG = kG;
		config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

		config.MotionMagic.MotionMagicCruiseVelocity =
				kMotionMagicCruiseVelocityDegPerSec / 360.0;
		config.MotionMagic.MotionMagicAcceleration =
				kMotionMagicAccelerationDegPerSecSq / 360.0;

		config.ClosedLoopGeneral.ContinuousWrap = false;

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
				Units.degreesToRotations(kMaxAngleDeg);
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
				Units.degreesToRotations(kMinAngleDeg);

		PhoenixUtil.applyConfigWithRetry(motor, config, 5);
	}

	@Override
	public void periodic() {
		if (hardwareEnabled) {
			refreshSignals();
		}

		if (DriverStation.isDisabled()) {
			stop();
		}

		motorDisconnectedAlert.set(hardwareEnabled && !motorConnected);
		motorOverheatAlert.set(hardwareEnabled && temperatureCelsius >= kMaxTemperatureCelsius);

		Logger.recordOutput("Hood/HardwareEnabled", hardwareEnabled);
		Logger.recordOutput("Hood/MotorConnected", motorConnected);
		Logger.recordOutput("Hood/EncodersZeroed", encodersZeroed);
		Logger.recordOutput("Hood/ControlMode", controlMode);
		Logger.recordOutput("Hood/RequestedSetpointDeg", requestedSetpointDeg, "degrees");
		Logger.recordOutput("Hood/ClampedSetpointDeg", clampedSetpointDeg, "degrees");
		Logger.recordOutput(
				"Hood/ClampedSetpointRotations",
				Units.degreesToRotations(clampedSetpointDeg),
				"rotations");
		Logger.recordOutput("Hood/PositionRotations", measuredPositionRotations, "rotations");
		Logger.recordOutput("Hood/PositionDeg", measuredAngleDeg, "degrees");
		Logger.recordOutput(
				"Hood/VelocityRotationsPerSec", measuredVelocityRotationsPerSec, "rotations_per_second");
		Logger.recordOutput("Hood/VelocityDegPerSec", measuredVelocityDegPerSec, "degrees_per_second");
		Logger.recordOutput("Hood/PositionErrorDeg", clampedSetpointDeg - measuredAngleDeg, "degrees");
		Logger.recordOutput("Hood/AppliedVolts", appliedVolts, "volts");
		Logger.recordOutput("Hood/TemperatureCelsius", temperatureCelsius, "celsius");
		Logger.recordOutput("Hood/NearSetpoint", isNearSetpoint());
		Logger.recordOutput("Hood/AtForwardLimit", getAngle() >= kMaxAngleDeg - 1.0);
		Logger.recordOutput("Hood/AtReverseLimit", getAngle() <= kMinAngleDeg + 1.0);
	}

	private void refreshSignals() {
		motorConnected = connectedDebouncer.calculate(BaseStatusSignal.refreshAll(
						motorVoltageSignal,
						velocitySignal,
						positionSignal,
						temperatureSignal)
				.isOK());

		measuredPositionRotations = positionSignal.getValueAsDouble();
		measuredVelocityRotationsPerSec = velocitySignal.getValueAsDouble();
		measuredAngleDeg = Units.rotationsToDegrees(measuredPositionRotations);
		measuredVelocityDegPerSec = Units.rotationsToDegrees(measuredVelocityRotationsPerSec);
		appliedVolts = motorVoltageSignal.getValueAsDouble();
		temperatureCelsius = temperatureSignal.getValueAsDouble();
	}

	public void setHoodAngle(double angleDeg) {
		requestedSetpointDeg = angleDeg;
		clampedSetpointDeg = MathUtil.clamp(angleDeg, kMinAngleDeg, kMaxAngleDeg);
		controlMode = ControlMode.MOTION_MAGIC;

		if (hardwareEnabled) {
			motor.setControl(
					motionMagicRequest.withPosition(Units.degreesToRotations(clampedSetpointDeg)));
		} else {
			measuredPositionRotations = Units.degreesToRotations(clampedSetpointDeg);
			measuredVelocityRotationsPerSec = 0.0;
			measuredAngleDeg = clampedSetpointDeg;
			measuredVelocityDegPerSec = 0.0;
		}
	}

	public void setVoltage(double volts) {
		controlMode = ControlMode.VOLTAGE;
		requestedSetpointDeg = getAngle();
		clampedSetpointDeg = requestedSetpointDeg;
		appliedVolts = volts;

		if (hardwareEnabled) {
			motor.setControl(voltageRequest.withOutput(volts));
		}
	}

	public void stop() {
		controlMode = ControlMode.NEUTRAL;
		appliedVolts = 0.0;
		if (hardwareEnabled) {
			motor.setControl(neutralRequest);
		}
	}

	public double getAngle() {
		return MathUtil.clamp(measuredAngleDeg, kMinAngleDeg, kMaxAngleDeg);
	}

	public boolean isNearSetpoint() {
		return Math.abs(clampedSetpointDeg - getAngle()) <= kAngleToleranceDeg;
	}

	public double getLatestSetpoint() {
		return clampedSetpointDeg;
	}

	public void zeroEncoders() {
		if (hardwareEnabled) {
			motor.setPosition(Units.degreesToRotations(kHomeAngleDeg));
		}
		measuredPositionRotations = Units.degreesToRotations(kHomeAngleDeg);
		measuredVelocityRotationsPerSec = 0.0;
		measuredAngleDeg = kHomeAngleDeg;
		measuredVelocityDegPerSec = 0.0;
		requestedSetpointDeg = kHomeAngleDeg;
		clampedSetpointDeg = kHomeAngleDeg;
		encodersZeroed = true;
	}

	public boolean areEncodersZeroed() {
		return encodersZeroed;
	}

	public Command sysIdQuasistatic(Direction direction) {
		return routine.quasistatic(direction);
	}

	public Command sysIdDynamic(Direction direction) {
		return routine.dynamic(direction);
	}
}
