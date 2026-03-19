package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kD;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kFollowerMotorID;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kFollowerOpposesLeader;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kIsFOC;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kLeadMotorID;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kLeadMotorInverted;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMotorCANBus;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMotorSupplyLimitAmps;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kP;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kS;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kV;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/**
 * TalonFX flywheel IO for two Kraken X60 motors on the same flywheel.
 */
public class FlywheelIOTalonFX implements FlywheelIO {
	private final TalonFX m_leadMotor;
	private final TalonFX m_followerMotor;

	// Control objects
	private final VelocityVoltage m_velocityRequest =
			new VelocityVoltage(0.0).withEnableFOC(kIsFOC).withSlot(0);
	private final VoltageOut m_voltageRequest =
			new VoltageOut(0.0).withEnableFOC(kIsFOC);

	// Status signals from the lead motor
	private final StatusSignal<Voltage> m_voltageSignal;
	private final StatusSignal<AngularVelocity> m_velocitySignal;
	private final StatusSignal<Angle> m_positionSignal;
	private final StatusSignal<Current> m_supplyCurrentSignal;
	private final StatusSignal<Current> m_statorCurrentSignal;
	private final StatusSignal<Temperature> m_tempSignal;

	public FlywheelIOTalonFX() {
		m_leadMotor = new TalonFX(kLeadMotorID, new CANBus(kMotorCANBus));
		m_followerMotor = new TalonFX(kFollowerMotorID, new CANBus(kMotorCANBus));

		// Lead motor config
		TalonFXConfiguration leadConfig = new TalonFXConfiguration();
		leadConfig.CurrentLimits.StatorCurrentLimit = kMotorStatorLimitAmps;
		leadConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		leadConfig.CurrentLimits.SupplyCurrentLimit = kMotorSupplyLimitAmps;
		leadConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

		leadConfig.MotorOutput.DutyCycleNeutralDeadband = 0.03;
		leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		leadConfig.MotorOutput.Inverted = kLeadMotorInverted
				? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;

		leadConfig.Feedback.SensorToMechanismRatio = kGearboxReduction;

		// Phoenix velocity units are rotations/sec, not RPM.
		leadConfig.Slot0.kP = kP * 60.0;
		leadConfig.Slot0.kD = kD * 60.0;
		leadConfig.Slot0.kS = kS;
		leadConfig.Slot0.kV = kV * 60.0;

		PhoenixUtil.applyConfigWithRetry(m_leadMotor, leadConfig, 3);

		// Follower motor config
		TalonFXConfiguration followerConfig = new TalonFXConfiguration();
		followerConfig.CurrentLimits.StatorCurrentLimit = kMotorStatorLimitAmps;
		followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		followerConfig.CurrentLimits.SupplyCurrentLimit = kMotorSupplyLimitAmps;
		followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

		followerConfig.MotorOutput.DutyCycleNeutralDeadband = 0.03;
		followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		followerConfig.Feedback.SensorToMechanismRatio = kGearboxReduction;

		PhoenixUtil.applyConfigWithRetry(m_followerMotor, followerConfig, 3);

		// Set follower
m_followerMotor.setControl(
    new Follower(
        m_leadMotor.getDeviceID(),
        kFollowerOpposesLeader
            ? MotorAlignmentValue.Opposed
            : MotorAlignmentValue.Aligned
    )
);
		// Signals from lead motor
		m_voltageSignal = m_leadMotor.getMotorVoltage();
		m_velocitySignal = m_leadMotor.getVelocity();
		m_positionSignal = m_leadMotor.getPosition();
		m_supplyCurrentSignal = m_leadMotor.getSupplyCurrent();
		m_statorCurrentSignal = m_leadMotor.getStatorCurrent();
		m_tempSignal = m_leadMotor.getDeviceTemp();

		BaseStatusSignal.setUpdateFrequencyForAll(
				100.0,
				m_voltageSignal,
				m_velocitySignal,
				m_positionSignal,
				m_supplyCurrentSignal,
				m_statorCurrentSignal,
				m_tempSignal
		);

		ParentDevice.optimizeBusUtilizationForAll(m_leadMotor, m_followerMotor);
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		inputs.motorConnected = BaseStatusSignal.refreshAll(
				m_voltageSignal,
				m_velocitySignal,
				m_positionSignal,
				m_supplyCurrentSignal,
				m_statorCurrentSignal,
				m_tempSignal
		).isOK();

		inputs.appliedVoltage = m_voltageSignal.getValue().in(Volts);
		inputs.rpm = m_velocitySignal.getValue().in(RPM);
		inputs.positionRevs = m_positionSignal.getValue().in(Revolutions);
		inputs.supplyCurrentAmps = Math.abs(m_supplyCurrentSignal.getValue().in(Amps));
		inputs.statorCurrentAmps = Math.abs(m_statorCurrentSignal.getValue().in(Amps));
		inputs.temperatureCelsius = m_tempSignal.getValue().in(Celsius);
	}

	@Override
	public void runVolts(double volts) {
		m_leadMotor.setControl(m_voltageRequest.withOutput(volts));
	}

	@Override
	public void runVelocity(double rpm) {
		m_leadMotor.setControl(m_velocityRequest.withVelocity(rpm / 60.0));
	}

	@Override
	public void zeroEncoders() {
		m_leadMotor.setPosition(0.0);
	}

	@Override
	public void stop() {
		m_leadMotor.stopMotor();
	}
}