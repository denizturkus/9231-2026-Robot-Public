package frc.robot.subsystems.indexer.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FeederIOTalonFX implements FeederIO {
    private static final int kMaxAttempts = 5;

    private final Debouncer m_motorConnectedDebouncer =
            new Debouncer(0.5, DebounceType.kFalling);

    private final TalonFX m_talon;
    private final StatusSignal<Voltage> m_volts;
    private final StatusSignal<Angle> m_position;
    private final StatusSignal<AngularVelocity> m_velocity;
    private final StatusSignal<Current> m_supplyCurrent;
    private final StatusSignal<Current> m_statorCurrent;
    private final StatusSignal<Temperature> m_temperature;

    private final VoltageOut m_voltageControl;

    public FeederIOTalonFX() {
         m_talon = new TalonFX(
                FeederConstants.kMotorID,
                new CANBus(FeederConstants.kMotorCANBus)
        );

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = FeederConstants.kMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = FeederConstants.kMotorSupplyLimitAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = FeederConstants.kMotorStatorLimitAmps;
        config.MotorOutput.PeakForwardDutyCycle = FeederConstants.kPeakForwardDutyCycle;
        config.MotorOutput.PeakReverseDutyCycle = FeederConstants.kPeakReverseDutyCycle;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.DutyCycleNeutralDeadband = 0.03;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < kMaxAttempts; i++) {
            status = m_talon.getConfigurator().apply(config);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println("Failed to apply feeder TalonFX config: " + status);
        }

        m_voltageControl = new VoltageOut(0.0).withEnableFOC(FeederConstants.kIsFOC);
        
        m_volts = m_talon.getMotorVoltage();
        m_position = m_talon.getPosition();
        m_velocity = m_talon.getVelocity();
        m_supplyCurrent = m_talon.getSupplyCurrent();
        m_statorCurrent = m_talon.getStatorCurrent();
        m_temperature = m_talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                m_volts, m_position, m_velocity, m_supplyCurrent, m_statorCurrent, m_temperature
        );

        m_talon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.motorConnected = m_motorConnectedDebouncer.calculate(
                BaseStatusSignal.refreshAll(
                        m_volts, m_position, m_velocity,
                        m_supplyCurrent, m_statorCurrent, m_temperature
                ).isOK()
        );

        inputs.appliedVoltage = m_volts.getValue().in(Volts);
        inputs.positionRevs = m_position.getValue().in(Revolutions);
        inputs.rpm = m_velocity.getValue().in(RPM);
        inputs.supplyCurrentAmps = Math.abs(m_supplyCurrent.getValue().in(Amps));
        inputs.statorCurrentAmps = Math.abs(m_statorCurrent.getValue().in(Amps));
        inputs.temperatureCelsius = m_temperature.getValue().in(Celsius);
    }

    @Override
    public void runVolts(double volts) {
        m_talon.setControl(m_voltageControl.withOutput(volts));
    }
}