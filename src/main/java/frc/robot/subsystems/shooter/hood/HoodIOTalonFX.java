package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.kD;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kG;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kHoodCalibrationAngle;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kIsFOC;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMinAngleDeg;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotionMagicAccelerationDegPerSecSq;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotionMagicMaxVelocityDegPerSec;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotorID;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotorInverted;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kMotorSupplyLimitAmps;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kP;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kS;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kV;

import frc.robot.util.PhoenixUtil;
import com.ctre.phoenix6.BaseStatusSignal;
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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * The IO hardware implementation for hood hardware interacions with the TalonFX.
 */
public class HoodIOTalonFX implements HoodIO {
	private TalonFX m_talon;

	private Debouncer m_motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

	private VoltageOut m_voltsOut = new VoltageOut(0).withEnableFOC(kIsFOC);
	private MotionMagicVoltage m_positionVoltage = new MotionMagicVoltage(Units.degreesToRotations(kMinAngleDeg))
			.withEnableFOC(kIsFOC);

	private StatusSignal<Voltage> m_voltsSignal;
	private StatusSignal<AngularVelocity> m_rpmSignal;
	private StatusSignal<Angle> m_positionSignal;
	private StatusSignal<Current> m_supplySignal, m_statorSignal;
	private StatusSignal<Temperature> m_tempSignal;

	/**
	 * Constructs a new HoodIOKraken.
	 */
	public HoodIOTalonFX() {
		m_talon = new TalonFX(kMotorID);

		// Configure motors
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.CurrentLimits.StatorCurrentLimit = kMotorStatorLimitAmps;
		motorConfig.CurrentLimits.SupplyCurrentLimit = kMotorSupplyLimitAmps;
		motorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.03;
		motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		motorConfig.MotorOutput.Inverted = kMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		motorConfig.Feedback.SensorToMechanismRatio = kGearboxReduction;
		motorConfig.Slot0.kP = kP * 360.0;
		motorConfig.Slot0.kD = kD * 360.0;
		motorConfig.Slot0.kS = kS;
		motorConfig.Slot0.kV = kV * 360.0;
		motorConfig.Slot0.kG = kG;
		motorConfig.MotionMagic.MotionMagicCruiseVelocity = kMotionMagicMaxVelocityDegPerSec / 360.0;
		motorConfig.MotionMagic.MotionMagicAcceleration = kMotionMagicAccelerationDegPerSecSq / 360.0;
		motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
		motorConfig.ClosedLoopGeneral.ContinuousWrap = true;

		PhoenixUtil.applyConfigWithRetry(m_talon, motorConfig, 3);

		// Status signals
		m_voltsSignal = m_talon.getMotorVoltage();
		m_rpmSignal = m_talon.getVelocity();
		m_positionSignal = m_talon.getPosition();
		m_supplySignal = m_talon.getSupplyCurrent();
		m_statorSignal = m_talon.getStatorCurrent();
		m_tempSignal = m_talon.getDeviceTemp();
		BaseStatusSignal.setUpdateFrequencyForAll(100, m_voltsSignal, m_rpmSignal, m_positionSignal, m_supplySignal,
				m_statorSignal, m_tempSignal);
		ParentDevice.optimizeBusUtilizationForAll(m_talon);
	}

	@Override
	public void updateInputs(HoodIOInputs inputs) {
		inputs.motorConnected = m_motorConnectedDebouncer.calculate(BaseStatusSignal
				.refreshAll(m_voltsSignal, m_rpmSignal, m_positionSignal, m_supplySignal, m_statorSignal, m_tempSignal)
				.isOK());
		inputs.appliedVoltage = m_voltsSignal.getValueAsDouble();
		inputs.rpm = m_rpmSignal.getValueAsDouble() * 60.0;
		inputs.positionDegrees = MathUtil.inputModulus(Units.rotationsToDegrees(m_positionSignal.getValueAsDouble()),
				-180, 180);
		inputs.supplyCurrentAmps = Math.abs(m_supplySignal.getValueAsDouble());
		inputs.statorCurrentAmps = Math.abs(m_statorSignal.getValueAsDouble());
		inputs.temperatureCelsius = m_tempSignal.getValueAsDouble();
	}

	@Override
	public void runVolts(double volts) {
		m_talon.setControl(m_voltsOut.withOutput(volts));
	}

	@Override
	public void runPosition(double degs) {
		double revs = Units.degreesToRotations(degs);
		m_talon.setControl(m_positionVoltage.withPosition(revs));
	}

	@Override
	public void zeroEncoders() {
		m_talon.setPosition(Units.degreesToRotations(kHoodCalibrationAngle));
	}

	@Override
	public void stop() {
		m_talon.stopMotor();
	}
}
