package frc.robot.subsystems.indexer.hopper;

import static frc.robot.subsystems.indexer.hopper.HopperConstants.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(kMotorId, kCANBus);

    private final VoltageOut m_voltageRequest = new VoltageOut(0.0);
    private final NeutralOut m_neutralRequest = new NeutralOut();

    private final StatusSignal<Angle> m_position = m_motor.getPosition();
    private final StatusSignal<AngularVelocity> m_velocity = m_motor.getVelocity();
    private final StatusSignal<Voltage> m_motorVoltage = m_motor.getMotorVoltage();
    private final StatusSignal<Current> m_supplyCurrent = m_motor.getSupplyCurrent();
    private final StatusSignal<Current> m_statorCurrent = m_motor.getStatorCurrent();
    private final StatusSignal<Temperature> m_deviceTemp = m_motor.getDeviceTemp();
    private final StatusSignal<ReverseLimitValue> m_reverseLimit = m_motor.getReverseLimit();

    private double m_setpointVolts = 0.0;

    public HopperSubsystem() {
        configureMotor();
        configureSignals();
        stop();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = kMotorInverted;
        config.MotorOutput.NeutralMode = kNeutralMode;

        config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimitAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = kEnableSupplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimitAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = kEnableStatorCurrentLimit;


        m_motor.getConfigurator().apply(config);
    }

    private void configureSignals() {
        BaseStatusSignal.setUpdateFrequencyForAll(
            kStatusSignalFrequencyHz,
            m_position,
            m_velocity,
            m_motorVoltage,
            m_supplyCurrent,
            m_statorCurrent,
            m_deviceTemp,
            m_reverseLimit
        );

        m_motor.optimizeBusUtilization();
    }

    /** Runs the hopper in the feeding direction. */
    public void feed() {
        runVoltage(kFeedVoltage);
    }

    /** Runs the hopper in reverse to clear jams. */
    public void reverse() {
        runVoltage(kReverseVoltage);
    }

    private void runVoltage(double volts) {
        m_setpointVolts = volts;
        m_motor.setControl(m_voltageRequest.withOutput(volts));
    }

    /** Stops the motor */
    public void stop() {
        m_setpointVolts = kStopVoltage;
        m_motor.setControl(m_neutralRequest);
    }

    /** Returns the hopper position (rotations). */
    public double getPositionRotations() {
        return m_position.getValueAsDouble();
    }

    /** Returns the rotor velocity (rotations/sec). */
    public double getVelocityRps() {
        return m_velocity.getValueAsDouble();
    }

    /** Returns the voltage applied to the motor. */
    public double getAppliedVolts() {
        return m_motorVoltage.getValueAsDouble();
    }

    /** Supply current (A). */
    public double getSupplyCurrentAmps() {
        return m_supplyCurrent.getValueAsDouble();
    }

    /** Stator current (A). */
    public double getStatorCurrentAmps() {
        return m_statorCurrent.getValueAsDouble();
    }

    /** Temperature (C). */
    public double getTemperatureCelsius() {
        return m_deviceTemp.getValueAsDouble();
    }

    //are the motors really moving, or are we stalled/jammed? Useful for debugging and for deciding when to trigger a stall recovery routine.
    public boolean isMoving() {
        return Math.abs(getVelocityRps()) > 0.1;
    }

    //if the motor isn't moving but we're applying significant voltage and drawing a lot of current, it's likely that the hopper is jammed.
    public boolean isLikelyJammed() {
        return Math.abs(getVelocityRps()) < 0.2
            && getStatorCurrentAmps() > kJamCurrentThresholdAmps
            && Math.abs(m_setpointVolts) > 1.0;
    }

    public boolean isOverTemp() {
        return getTemperatureCelsius() >= kHighTempThresholdCelsius;
    }

    @Override
    public void periodic() {
                BaseStatusSignal.refreshAll(
            m_position,
            m_velocity,
            m_motorVoltage,
            m_supplyCurrent,
            m_statorCurrent,
            m_deviceTemp,
            m_reverseLimit
        );

        

        // AdvantageKit output logging
        Logger.recordOutput("Hopper/SetpointVolts", m_setpointVolts, "volts");
        Logger.recordOutput("Hopper/PositionRotations", getPositionRotations(), "rotations");
        Logger.recordOutput("Hopper/VelocityRps", getVelocityRps(), "rotations_per_second");
        Logger.recordOutput("Hopper/AppliedVolts", getAppliedVolts(), "volts");
        Logger.recordOutput("Hopper/SupplyCurrentAmps", getSupplyCurrentAmps(), "amps");
        Logger.recordOutput("Hopper/StatorCurrentAmps", getStatorCurrentAmps(), "amps");
        Logger.recordOutput("Hopper/TemperatureCelsius", getTemperatureCelsius(), "celsius");
        Logger.recordOutput("Hopper/IsMoving", isMoving());
        Logger.recordOutput("Hopper/IsLikelyJammed", isLikelyJammed());
        Logger.recordOutput("Hopper/IsOverTemp", isOverTemp());
    }
}