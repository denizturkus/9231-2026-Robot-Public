package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kD;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kFollowerMotorID;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kFollowerOpposesLeader;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kI;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kLeadMotorID;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kLeadMotorInverted;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMaxAllowedRPM;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMaxTemperatureCelsius;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMotorCANBus;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kMotorSupplyLimitAmps;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kP;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kS;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kStatusSignalFrequencyHz;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kSysIdTimeout;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kSysIdVoltageRampRate;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kSysIdVoltageStep;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kUseFOC;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kV;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kVelocityHysteresisToleranceRPM;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.kVelocityToleranceRPM;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Direct TalonFX flywheel subsystem for a lead/follower Kraken X60 pair. */
public class FlywheelSubsystem extends SubsystemBase {
    private static final String kTuningKeyPrefix = "FlywheelTuning/";

    private enum ControlMode {
        NEUTRAL,
        VOLTAGE,
        VELOCITY
    }

    private static final double kMinimumClosedLoopReadyRpm = 1.0;
    private static final int kVelocityFilterWindowSamples = 5;
    private static final double kReadyDebounceSeconds = 0.12;
    private static final double kGainUpdateEpsilon = 1e-6;

    private final boolean hardwareEnabled;
    private final TalonFX leadMotor;
    private final TalonFX followerMotor;

    private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(kUseFOC);
    private final VelocityVoltage velocityRequest =
            new VelocityVoltage(0.0).withEnableFOC(kUseFOC).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    private final StatusSignal<Voltage> leadMotorVoltageSignal;
    private final StatusSignal<AngularVelocity> leadVelocitySignal;
    private final StatusSignal<Angle> leadPositionSignal;
    private final StatusSignal<Temperature> leadTemperatureSignal;
    private final StatusSignal<Voltage> followerMotorVoltageSignal;
    private final StatusSignal<Temperature> followerTemperatureSignal;

    private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
    private final Debouncer nearSetpointDebouncer =
            new Debouncer(kReadyDebounceSeconds, DebounceType.kRising);
    private final LinearFilter velocityFilter =
            LinearFilter.movingAverage(kVelocityFilterWindowSamples);
    private final Alert motorDisconnectedAlert =
            new Alert(
                    "Flywheel motor disconnected! (Lead ID: "
                            + kLeadMotorID
                            + ", Follower ID: "
                            + kFollowerMotorID
                            + ")",
                    AlertType.kError);
    private final Alert motorOverheatAlert =
            new Alert(
                    "Flywheel motor overheating! (Lead ID: "
                            + kLeadMotorID
                            + ", Follower ID: "
                            + kFollowerMotorID
                            + ")",
                    AlertType.kWarning);
    private final SysIdRoutine routine;
    private final LoggedNetworkNumber kPTunable =
            new LoggedNetworkNumber(kTuningKeyPrefix + "kP", kP);
    private final LoggedNetworkNumber kITunable =
            new LoggedNetworkNumber(kTuningKeyPrefix + "kI", kI);
    private final LoggedNetworkNumber kDTunable =
            new LoggedNetworkNumber(kTuningKeyPrefix + "kD", kD);
    private final LoggedNetworkNumber kSTunable =
            new LoggedNetworkNumber(kTuningKeyPrefix + "kS", kS);
    private final LoggedNetworkNumber kVTunable =
            new LoggedNetworkNumber(kTuningKeyPrefix + "kV", kV);
    private final LoggedNetworkNumber readyToleranceTunable =
            new LoggedNetworkNumber(
                    kTuningKeyPrefix + "ReadyToleranceRPM",
                    kVelocityToleranceRPM);
    private final LoggedNetworkNumber readyHysteresisToleranceTunable =
            new LoggedNetworkNumber(
                    kTuningKeyPrefix + "ReadyHysteresisToleranceRPM",
                    kVelocityHysteresisToleranceRPM);

    private double measuredPositionRotations = 0.0;
    private double measuredVelocityRpm = 0.0;
    private double filteredVelocityRpm = 0.0;
    private double leadAppliedVolts = 0.0;
    private double followerAppliedVolts = 0.0;
    private double leadTemperatureCelsius = 0.0;
    private double followerTemperatureCelsius = 0.0;
    private double configuredKP = kP;
    private double configuredKI = kI;
    private double configuredKD = kD;
    private double configuredKS = kS;
    private double configuredKV = kV;
    private double readyToleranceRpm = kVelocityToleranceRPM;
    private double readyHysteresisToleranceRpm = kVelocityHysteresisToleranceRPM;

    private double requestedSetpointRPM = 0.0;
    private double clampedSetpointRPM = 0.0;
    private boolean motorConnected = false;
    private boolean encodersZeroed = false;
    private boolean nearSetpoint = false;
    private ControlMode controlMode = ControlMode.NEUTRAL;

    public FlywheelSubsystem() {
        setName("Flywheel");
        hardwareEnabled = Constants.currentMode == Constants.Mode.REAL;
        refreshTuningValues();

        if (hardwareEnabled) {
            leadMotor =
                    kMotorCANBus.isEmpty()
                            ? new TalonFX(kLeadMotorID)
                            : new TalonFX(kLeadMotorID, kMotorCANBus);
            followerMotor =
                    kMotorCANBus.isEmpty()
                            ? new TalonFX(kFollowerMotorID)
                            : new TalonFX(kFollowerMotorID, kMotorCANBus);
            configureMotors();

            leadMotorVoltageSignal = leadMotor.getMotorVoltage();
            leadVelocitySignal = leadMotor.getVelocity();
            leadPositionSignal = leadMotor.getPosition();
            leadTemperatureSignal = leadMotor.getDeviceTemp();
            followerMotorVoltageSignal = followerMotor.getMotorVoltage();
            followerTemperatureSignal = followerMotor.getDeviceTemp();

            BaseStatusSignal.setUpdateFrequencyForAll(
                    kStatusSignalFrequencyHz,
                    leadMotorVoltageSignal,
                    leadVelocitySignal,
                    leadPositionSignal,
                    leadTemperatureSignal,
                    followerMotorVoltageSignal,
                    followerTemperatureSignal);
            ParentDevice.optimizeBusUtilizationForAll(leadMotor, followerMotor);

            zeroEncoders();
            refreshSignals();
        } else {
            leadMotor = null;
            followerMotor = null;
            leadMotorVoltageSignal = null;
            leadVelocitySignal = null;
            leadPositionSignal = null;
            leadTemperatureSignal = null;
            followerMotorVoltageSignal = null;
            followerTemperatureSignal = null;
            motorConnected = true;
        }

        routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Volts.of(kSysIdVoltageRampRate).per(Seconds),
                                Volts.of(kSysIdVoltageStep),
                                Seconds.of(kSysIdTimeout)),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> setVoltage(voltage.in(Volts)),
                                this::logSysIdData,
                                this));
    }

    private void configureMotors() {
        TalonFXConfiguration leadConfig = new TalonFXConfiguration();
        leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leadConfig.MotorOutput.Inverted =
                kLeadMotorInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        leadConfig.CurrentLimits.SupplyCurrentLimit = kMotorSupplyLimitAmps;
        leadConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leadConfig.CurrentLimits.StatorCurrentLimit = kMotorStatorLimitAmps;
        leadConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leadConfig.Feedback.SensorToMechanismRatio = kGearboxReduction;
        leadConfig.Slot0.kP = configuredKP * 60.0;
        leadConfig.Slot0.kI = configuredKI * 60.0;
        leadConfig.Slot0.kD = configuredKD * 60.0;
        leadConfig.Slot0.kS = configuredKS;
        leadConfig.Slot0.kV = configuredKV * 60.0;

        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        followerConfig.CurrentLimits.SupplyCurrentLimit = kMotorSupplyLimitAmps;
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerConfig.CurrentLimits.StatorCurrentLimit = kMotorStatorLimitAmps;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerConfig.Feedback.SensorToMechanismRatio = kGearboxReduction;

        PhoenixUtil.applyConfigWithRetry(leadMotor, leadConfig, 5);
        PhoenixUtil.applyConfigWithRetry(followerMotor, followerConfig, 5);

        followerMotor.setControl(
                new Follower(
                        leadMotor.getDeviceID(),
                        kFollowerOpposesLeader
                                ? MotorAlignmentValue.Opposed
                                : MotorAlignmentValue.Aligned));
    }

    @Override
    public void periodic() {
        if (hardwareEnabled) {
            refreshSignals();
        }
        updateTunableGains();

        filteredVelocityRpm = velocityFilter.calculate(measuredVelocityRpm);
        double filteredVelocityErrorRpm = clampedSetpointRPM - filteredVelocityRpm;
        boolean closedLoopReadyCheckEnabled =
                controlMode == ControlMode.VELOCITY
                        && Math.abs(clampedSetpointRPM) > kMinimumClosedLoopReadyRpm;
        boolean insideReadyBand = Math.abs(filteredVelocityErrorRpm) <= readyToleranceRpm;
        boolean insideHysteresisBand =
                Math.abs(filteredVelocityErrorRpm) <= readyHysteresisToleranceRpm;
        nearSetpoint =
                nearSetpointDebouncer.calculate(
                        closedLoopReadyCheckEnabled
                                && (nearSetpoint ? insideHysteresisBand : insideReadyBand));

        if (DriverStation.isDisabled()) {
            stop();
        }

        motorDisconnectedAlert.set(hardwareEnabled && !motorConnected);
        motorOverheatAlert.set(
                hardwareEnabled
                        && Math.max(leadTemperatureCelsius, followerTemperatureCelsius)
                                >= kMaxTemperatureCelsius);

        Logger.recordOutput("Flywheel/HardwareEnabled", hardwareEnabled);
        Logger.recordOutput("Flywheel/MotorConnected", motorConnected);
        Logger.recordOutput("Flywheel/EncodersZeroed", encodersZeroed);
        Logger.recordOutput("Flywheel/ControlMode", controlMode);
        Logger.recordOutput("Flywheel/RequestedSetpointRPM", requestedSetpointRPM, "revolutions_per_minute");
        Logger.recordOutput("Flywheel/ClampedSetpointRPM", clampedSetpointRPM, "revolutions_per_minute");
        Logger.recordOutput("Flywheel/PositionRotations", measuredPositionRotations, "rotations");
        Logger.recordOutput("Flywheel/VelocityRPM", measuredVelocityRpm, "revolutions_per_minute");
        Logger.recordOutput(
                "Flywheel/FilteredVelocityRPM",
                filteredVelocityRpm,
                "revolutions_per_minute");
        Logger.recordOutput(
                "Flywheel/VelocityErrorRPM",
                clampedSetpointRPM - measuredVelocityRpm,
                "revolutions_per_minute");
        Logger.recordOutput(
                "Flywheel/FilteredVelocityErrorRPM",
                filteredVelocityErrorRpm,
                "revolutions_per_minute");
        Logger.recordOutput("Flywheel/InsideReadyBand", insideReadyBand);
        Logger.recordOutput("Flywheel/InsideHysteresisBand", insideHysteresisBand);
        Logger.recordOutput("Flywheel/LeadAppliedVolts", leadAppliedVolts, "volts");
        Logger.recordOutput("Flywheel/FollowerAppliedVolts", followerAppliedVolts, "volts");
        Logger.recordOutput("Flywheel/LeadTemperatureCelsius", leadTemperatureCelsius, "celsius");
        Logger.recordOutput("Flywheel/FollowerTemperatureCelsius", followerTemperatureCelsius, "celsius");
        Logger.recordOutput("Flywheel/NearSetpoint", isNearSetpoint());
        Logger.recordOutput("Flywheel/ConfiguredKP", configuredKP);
        Logger.recordOutput("Flywheel/ConfiguredKI", configuredKI);
        Logger.recordOutput("Flywheel/ConfiguredKD", configuredKD);
        Logger.recordOutput("Flywheel/ConfiguredKS", configuredKS);
        Logger.recordOutput("Flywheel/ConfiguredKV", configuredKV);
        Logger.recordOutput("Flywheel/ReadyToleranceRPM", readyToleranceRpm, "revolutions_per_minute");
        Logger.recordOutput(
                "Flywheel/ReadyHysteresisToleranceRPM",
                readyHysteresisToleranceRpm,
                "revolutions_per_minute");
    }

    private void refreshSignals() {
        motorConnected =
                connectedDebouncer.calculate(
                        BaseStatusSignal.refreshAll(
                                        leadMotorVoltageSignal,
                                        leadVelocitySignal,
                                        leadPositionSignal,
                                        leadTemperatureSignal,
                                        followerMotorVoltageSignal,
                                        followerTemperatureSignal)
                                .isOK());

        measuredPositionRotations = leadPositionSignal.getValue().in(Revolutions);
        measuredVelocityRpm = leadVelocitySignal.getValue().in(RPM);
        leadAppliedVolts = leadMotorVoltageSignal.getValue().in(Volts);
        followerAppliedVolts = followerMotorVoltageSignal.getValue().in(Volts);
        leadTemperatureCelsius = leadTemperatureSignal.getValue().in(Celsius);
        followerTemperatureCelsius = followerTemperatureSignal.getValue().in(Celsius);
    }

    private void logSysIdData(SysIdRoutineLog log) {
        if (!hardwareEnabled) {
            return;
        }

        refreshSignals();
        log.motor("flywheel-lead")
                .voltage(leadMotorVoltageSignal.getValue())
                .angularPosition(leadPositionSignal.getValue())
                .angularVelocity(leadVelocitySignal.getValue());
    }

    private void refreshTuningValues() {
        configuredKP = kPTunable.get();
        configuredKI = kITunable.get();
        configuredKD = kDTunable.get();
        configuredKS = kSTunable.get();
        configuredKV = kVTunable.get();
        readyToleranceRpm = Math.max(0.0, readyToleranceTunable.get());
        readyHysteresisToleranceRpm =
                Math.max(readyToleranceRpm, readyHysteresisToleranceTunable.get());
    }

    private void updateTunableGains() {
        if (!Constants.kDoTuning) {
            return;
        }

        double newKP = kPTunable.get();
        double newKI = kITunable.get();
        double newKD = kDTunable.get();
        double newKS = kSTunable.get();
        double newKV = kVTunable.get();
        double newReadyToleranceRpm = Math.max(0.0, readyToleranceTunable.get());
        double newReadyHysteresisToleranceRpm =
                Math.max(newReadyToleranceRpm, readyHysteresisToleranceTunable.get());

        boolean gainsChanged =
                hasChanged(configuredKP, newKP)
                        || hasChanged(configuredKI, newKI)
                        || hasChanged(configuredKD, newKD)
                        || hasChanged(configuredKS, newKS)
                        || hasChanged(configuredKV, newKV);

        configuredKP = newKP;
        configuredKI = newKI;
        configuredKD = newKD;
        configuredKS = newKS;
        configuredKV = newKV;
        readyToleranceRpm = newReadyToleranceRpm;
        readyHysteresisToleranceRpm = newReadyHysteresisToleranceRpm;

        if (hardwareEnabled && gainsChanged) {
            applyClosedLoopGains();
        }
    }

    private void applyClosedLoopGains() {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = configuredKP * 60.0;
        slot0.kI = configuredKI * 60.0;
        slot0.kD = configuredKD * 60.0;
        slot0.kS = configuredKS;
        slot0.kV = configuredKV * 60.0;
        PhoenixUtil.tryUntilOk(5, () -> leadMotor.getConfigurator().apply(slot0));
    }

    private static boolean hasChanged(double currentValue, double newValue) {
        return Math.abs(currentValue - newValue) > kGainUpdateEpsilon;
    }

    public void setVelocity(double rpm) {
        double previousClampedSetpointRPM = clampedSetpointRPM;
        ControlMode previousControlMode = controlMode;
        requestedSetpointRPM = rpm;
        clampedSetpointRPM = MathUtil.clamp(rpm, -kMaxAllowedRPM, kMaxAllowedRPM);
        if (previousControlMode != ControlMode.VELOCITY
                || Math.abs(clampedSetpointRPM - previousClampedSetpointRPM)
                        > readyHysteresisToleranceRpm) {
            nearSetpoint = false;
        }
        controlMode = ControlMode.VELOCITY;

        if (hardwareEnabled) {
            leadMotor.setControl(velocityRequest.withVelocity(clampedSetpointRPM / 60.0));
        }
    }

    public void setVoltage(double volts) {
        controlMode = ControlMode.VOLTAGE;
        requestedSetpointRPM = getVelocity();
        clampedSetpointRPM = requestedSetpointRPM;
        nearSetpoint = false;
        leadAppliedVolts = volts;
        followerAppliedVolts = volts;

        if (hardwareEnabled) {
            leadMotor.setControl(voltageRequest.withOutput(volts));
        }
    }

    public void stop() {
        controlMode = ControlMode.NEUTRAL;
        requestedSetpointRPM = 0.0;
        clampedSetpointRPM = 0.0;
        nearSetpoint = false;
        leadAppliedVolts = 0.0;
        followerAppliedVolts = 0.0;

        if (hardwareEnabled) {
            leadMotor.setControl(neutralRequest);
        }
    }

    public boolean isNearSetpoint() {
        return nearSetpoint;
    }

    public double getLatestSetpoint() {
        return clampedSetpointRPM;
    }

    public double getVelocity() {
        return measuredVelocityRpm;
    }

    public void zeroEncoders() {
        if (hardwareEnabled) {
            leadMotor.setPosition(0.0);
            followerMotor.setPosition(0.0);
        }

        measuredPositionRotations = 0.0;
        measuredVelocityRpm = 0.0;
        filteredVelocityRpm = 0.0;
        requestedSetpointRPM = 0.0;
        clampedSetpointRPM = 0.0;
        nearSetpoint = false;
        encodersZeroed = true;
        stop();
    }

    public boolean areEncodersZeroed() {
        return encodersZeroed;
    }

    public List<ParentDevice> getOrchestraDevices() {
        if (!hardwareEnabled) {
            return List.of();
        }
        return List.of(leadMotor, followerMotor);
    }

    public Command sysIdQuasistatic(Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return routine.dynamic(direction);
    }
}
