package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.kArmCalibrationAngleDeg;
import static frc.robot.subsystems.intake.IntakeConstants.kArmGearboxReduction;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorCANBus;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorID;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorInverted;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorStatorLimitAmps;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorSupplyLimitAmps;
import static frc.robot.subsystems.intake.IntakeConstants.kD;
import static frc.robot.subsystems.intake.IntakeConstants.kG;
import static frc.robot.subsystems.intake.IntakeConstants.kIsFOC;
import static frc.robot.subsystems.intake.IntakeConstants.kMotionMagicAccelerationDegPerSecSq;
import static frc.robot.subsystems.intake.IntakeConstants.kMotionMagicMaxVelocityDegPerSec;
import static frc.robot.subsystems.intake.IntakeConstants.kP;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerGearboxReduction;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorCANBus;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorID;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorInverted;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorStatorLimitAmps;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorSupplyLimitAmps;
import static frc.robot.subsystems.intake.IntakeConstants.kS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import java.util.List;

/** The IO hardware implementation for intake hardware interacions with the TalonFX. */
public class IntakeIOTalonFX implements IntakeIO {
	private static final int kConfigApplyAttempts = 5;

	private TalonFX m_rollerTalon, m_armTalon;
	private final TalonFXConfiguration m_armMotorConfig = new TalonFXConfiguration();
	private double m_armMotionMagicCruiseVelocityDegPerSec = kMotionMagicMaxVelocityDegPerSec;
	private double m_armMotionMagicAccelerationDegPerSecSq = kMotionMagicAccelerationDegPerSecSq;

	private VoltageOut m_rollerVoltage = new VoltageOut(0.0).withEnableFOC(kIsFOC),
			m_armVoltage = new VoltageOut(0).withEnableFOC(kIsFOC);
	private MotionMagicVoltage m_armPosition = new MotionMagicVoltage(0.0).withEnableFOC(kIsFOC).withSlot(0);

	private StatusSignal<Voltage> m_rollerVoltsSignal;
	private StatusSignal<AngularVelocity> m_rollerRpmSignal;
	private StatusSignal<Current> m_rollerSupplySignal, m_rollerStatorSignal;
	private StatusSignal<Temperature> m_rollerTempSignal;

	private StatusSignal<Voltage> m_armVoltsSignal;
	private StatusSignal<AngularVelocity> m_armRpmSignal;
	private StatusSignal<Angle> m_armPositionSignal;
	private StatusSignal<Current> m_armSupplySignal, m_armStatorSignal;
	private StatusSignal<Temperature> m_armTempSignal;

	/**
	 * Constructs a new IntakeIOKraken.
	 */
	public IntakeIOTalonFX() {
		m_rollerTalon = new TalonFX(kRollerMotorID, new CANBus(kRollerMotorCANBus));
		m_armTalon = new TalonFX(kArmMotorID, new CANBus(kArmMotorCANBus));

		// Configure arm motor
		m_armMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		m_armMotorConfig.CurrentLimits.StatorCurrentLimit = kArmMotorStatorLimitAmps;
		m_armMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		m_armMotorConfig.CurrentLimits.SupplyCurrentLimit = kArmMotorSupplyLimitAmps;
		m_armMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.03;
		m_armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		m_armMotorConfig.MotorOutput.Inverted = kArmMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		m_armMotorConfig.Feedback.SensorToMechanismRatio = kArmGearboxReduction;
		m_armMotorConfig.Slot0.kS = kS;
		m_armMotorConfig.Slot0.kP = kP * 360.0;
		m_armMotorConfig.Slot0.kD = kD * 360.0;
		m_armMotorConfig.Slot0.kG = kG;
		m_armMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
		m_armMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
		updateArmMotionMagicConfig(
				kMotionMagicMaxVelocityDegPerSec,
				kMotionMagicAccelerationDegPerSecSq);

		PhoenixUtil.applyConfigWithRetry(m_armTalon, m_armMotorConfig, kConfigApplyAttempts);

		// Configure roller motor
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		motorConfig.CurrentLimits.StatorCurrentLimit = kRollerMotorStatorLimitAmps;
		motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		motorConfig.CurrentLimits.SupplyCurrentLimit = kRollerMotorSupplyLimitAmps;
		motorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.03;
		motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		motorConfig.MotorOutput.Inverted = kRollerMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		motorConfig.Feedback.SensorToMechanismRatio = kRollerGearboxReduction;

		PhoenixUtil.applyConfigWithRetry(m_rollerTalon, motorConfig, kConfigApplyAttempts);

		// Status signals
		m_rollerVoltsSignal = m_rollerTalon.getMotorVoltage();
		m_rollerRpmSignal = m_rollerTalon.getVelocity();
		m_rollerSupplySignal = m_rollerTalon.getSupplyCurrent();
		m_rollerStatorSignal = m_rollerTalon.getStatorCurrent();
		m_rollerTempSignal = m_rollerTalon.getDeviceTemp();

		m_armVoltsSignal = m_armTalon.getMotorVoltage();
		m_armRpmSignal = m_armTalon.getVelocity();
		m_armPositionSignal = m_armTalon.getPosition();
		m_armSupplySignal = m_armTalon.getSupplyCurrent();
		m_armStatorSignal = m_armTalon.getStatorCurrent();
		m_armTempSignal = m_armTalon.getDeviceTemp();

		BaseStatusSignal.setUpdateFrequencyForAll(100, m_rollerVoltsSignal, m_rollerRpmSignal, m_rollerSupplySignal,
				m_rollerStatorSignal, m_rollerTempSignal, m_armVoltsSignal, m_armRpmSignal, m_armPositionSignal,
				m_armSupplySignal, m_armStatorSignal, m_armTempSignal);
		ParentDevice.optimizeBusUtilizationForAll(m_rollerTalon, m_armTalon);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		inputs.rollerMotorConnected = BaseStatusSignal
				.refreshAll(m_rollerVoltsSignal, m_rollerRpmSignal, m_rollerSupplySignal, m_rollerStatorSignal,
						m_rollerTempSignal)
				.isOK();
		inputs.rollerAppliedVoltage = m_rollerVoltsSignal.getValue().in(Volts);
		inputs.rollerRpm = m_rollerRpmSignal.getValue().in(RPM);
		inputs.rollerSupplyCurrentAmps = m_rollerSupplySignal.getValue().in(Amps);
		inputs.rollerStatorCurrentAmps = m_rollerStatorSignal.getValue().in(Amps);
		inputs.rollerTemperatureCelsius = m_rollerTempSignal.getValue().in(Celsius);

		inputs.armMotorConnected = BaseStatusSignal
				.refreshAll(m_armVoltsSignal, m_armRpmSignal, m_armPositionSignal, m_armSupplySignal, m_armStatorSignal,
						m_armTempSignal)
				.isOK();
		inputs.armAppliedVoltage = m_armVoltsSignal.getValue().in(Volts);
		inputs.armRpm = m_armRpmSignal.getValue().in(RPM);
		inputs.armPositionDegrees = MathUtil
				.inputModulus(Units.rotationsToDegrees(m_armPositionSignal.getValue().in(Revolutions)), -180, 180);
		inputs.armSupplyCurrentAmps = m_armSupplySignal.getValue().in(Amps);
		inputs.armStatorCurrentAmps = m_armStatorSignal.getValue().in(Amps);
		inputs.armTemperatureCelsius = m_armTempSignal.getValue().in(Celsius);
	}

	@Override
	public void runRollerVolts(double volts) {
		m_rollerTalon.setControl(m_rollerVoltage.withOutput(volts));
	}

	@Override
	public void runArmVolts(double volts) {
		m_armTalon.setControl(m_armVoltage.withOutput(volts));
	}

	@Override
	public void stopRollers() {
		m_rollerTalon.stopMotor();
	}

	@Override
	public void runArmPosition(double degrees) {
		runArmPosition(
				degrees,
				m_armMotionMagicCruiseVelocityDegPerSec,
				m_armMotionMagicAccelerationDegPerSecSq);
	}

	@Override
	public void runArmPosition(double degrees, double cruiseVelocityDegPerSec, double accelerationDegPerSecSq) {
		applyArmMotionMagicProfileIfNeeded(cruiseVelocityDegPerSec, accelerationDegPerSecSq);
		m_armTalon.setControl(
				m_armPosition.withPosition(Units.degreesToRotations(MathUtil.inputModulus(degrees, -180, 180))));
	}

	@Override
	public void zeroArmEncoders() {
		m_armTalon.setPosition(Units.degreesToRotations(kArmCalibrationAngleDeg));
	}

	@Override
	public void stopArm() {
		m_armTalon.stopMotor();
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		return List.of(m_rollerTalon, m_armTalon);
	}

	private void applyArmMotionMagicProfileIfNeeded(double cruiseVelocityDegPerSec, double accelerationDegPerSecSq) {
		if (cruiseVelocityDegPerSec == m_armMotionMagicCruiseVelocityDegPerSec
				&& accelerationDegPerSecSq == m_armMotionMagicAccelerationDegPerSecSq) {
			return;
		}

		updateArmMotionMagicConfig(cruiseVelocityDegPerSec, accelerationDegPerSecSq);
		PhoenixUtil.applyConfigWithRetry(m_armTalon, m_armMotorConfig, kConfigApplyAttempts);
	}

	private void updateArmMotionMagicConfig(double cruiseVelocityDegPerSec, double accelerationDegPerSecSq) {
		m_armMotorConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocityDegPerSec / 360.0;
		m_armMotorConfig.MotionMagic.MotionMagicAcceleration = accelerationDegPerSecSq / 360.0;
		m_armMotionMagicCruiseVelocityDegPerSec = cruiseVelocityDegPerSec;
		m_armMotionMagicAccelerationDegPerSecSq = accelerationDegPerSecSq;
	}
}
