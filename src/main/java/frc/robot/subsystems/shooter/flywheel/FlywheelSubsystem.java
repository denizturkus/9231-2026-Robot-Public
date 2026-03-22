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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Direct TalonFX flywheel subsystem for a lead/follower Kraken X60 pair. */
public class FlywheelSubsystem extends SubsystemBase {
    private enum ControlMode {
        NEUTRAL,
        VOLTAGE,
        VELOCITY
    }

    private static final double kMinimumClosedLoopReadyRpm = 1.0;

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
    private final LoggedNetworkNumber tunableP =
            new LoggedNetworkNumber("LiveTuning/Flywheel/kP", kP);
    private final LoggedNetworkNumber tunableD =
            new LoggedNetworkNumber("LiveTuning/Flywheel/kD", kD);
    private final LoggedNetworkNumber tunableS =
            new LoggedNetworkNumber("LiveTuning/Flywheel/kS", kS);
    private final LoggedNetworkNumber tunableV =
            new LoggedNetworkNumber("LiveTuning/Flywheel/kV", kV);
    private final SysIdRoutine routine;

    private double measuredPositionRotations = 0.0;
    private double measuredVelocityRpm = 0.0;
    private double leadAppliedVolts = 0.0;
    private double followerAppliedVolts = 0.0;
    private double leadTemperatureCelsius = 0.0;
    private double followerTemperatureCelsius = 0.0;

    private double requestedSetpointRPM = 0.0;
    private double clampedSetpointRPM = 0.0;
    private boolean motorConnected = false;
    private boolean encodersZeroed = false;
    private ControlMode controlMode = ControlMode.NEUTRAL;
    private double appliedKP = kP;
    private double appliedKD = kD;
    private double appliedKS = kS;
    private double appliedKV = kV;

    public FlywheelSubsystem() {
        setName("Flywheel");
        hardwareEnabled = Constants.currentMode == Constants.Mode.REAL;

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
                                Seconds.of(kSysIdTimeout),
                                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> setVoltage(voltage.in(Volts)),
                                null,
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
        leadConfig.Slot0.kP = kP * 60.0;
        leadConfig.Slot0.kD = kD * 60.0;
        leadConfig.Slot0.kS = kS;
        leadConfig.Slot0.kV = kV * 60.0;

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
            updateTunableGains();
            refreshSignals();
        }

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
                "Flywheel/VelocityErrorRPM",
                clampedSetpointRPM - measuredVelocityRpm,
                "revolutions_per_minute");
        Logger.recordOutput("Flywheel/LeadAppliedVolts", leadAppliedVolts, "volts");
        Logger.recordOutput("Flywheel/FollowerAppliedVolts", followerAppliedVolts, "volts");
        Logger.recordOutput("Flywheel/LeadTemperatureCelsius", leadTemperatureCelsius, "celsius");
        Logger.recordOutput("Flywheel/FollowerTemperatureCelsius", followerTemperatureCelsius, "celsius");
        Logger.recordOutput("Flywheel/NearSetpoint", isNearSetpoint());
        Logger.recordOutput("Flywheel/Tuning/kP", appliedKP);
        Logger.recordOutput("Flywheel/Tuning/kD", appliedKD);
        Logger.recordOutput("Flywheel/Tuning/kS", appliedKS);
        Logger.recordOutput("Flywheel/Tuning/kV", appliedKV);
    }

    private void updateTunableGains() {
        double desiredP = tunableP.get();
        double desiredD = tunableD.get();
        double desiredS = tunableS.get();
        double desiredV = tunableV.get();

        if (Double.compare(desiredP, appliedKP) == 0
                && Double.compare(desiredD, appliedKD) == 0
                && Double.compare(desiredS, appliedKS) == 0
                && Double.compare(desiredV, appliedKV) == 0) {
            return;
        }

        Slot0Configs slot0Config = new Slot0Configs();
        slot0Config.kP = desiredP * 60.0;
        slot0Config.kD = desiredD * 60.0;
        slot0Config.kS = desiredS;
        slot0Config.kV = desiredV * 60.0;
        PhoenixUtil.tryUntilOk(5, () -> leadMotor.getConfigurator().apply(slot0Config));

        appliedKP = desiredP;
        appliedKD = desiredD;
        appliedKS = desiredS;
        appliedKV = desiredV;
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

    public void setVelocity(double rpm) {
        requestedSetpointRPM = rpm;
        clampedSetpointRPM = MathUtil.clamp(rpm, -kMaxAllowedRPM, kMaxAllowedRPM);
        controlMode = ControlMode.VELOCITY;

        if (hardwareEnabled) {
            leadMotor.setControl(velocityRequest.withVelocity(clampedSetpointRPM / 60.0));
        }
    }

    public void setVoltage(double volts) {
        controlMode = ControlMode.VOLTAGE;
        requestedSetpointRPM = getVelocity();
        clampedSetpointRPM = requestedSetpointRPM;
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
        leadAppliedVolts = 0.0;
        followerAppliedVolts = 0.0;

        if (hardwareEnabled) {
            leadMotor.setControl(neutralRequest);
        }
    }

    public boolean isNearSetpoint() {
        return controlMode == ControlMode.VELOCITY
                && Math.abs(clampedSetpointRPM) > kMinimumClosedLoopReadyRpm
                && MathUtil.isNear(clampedSetpointRPM, measuredVelocityRpm, kVelocityToleranceRPM);
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
        requestedSetpointRPM = 0.0;
        clampedSetpointRPM = 0.0;
        encodersZeroed = true;
        stop();
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
