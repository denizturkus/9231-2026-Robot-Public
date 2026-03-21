package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.kAngleToleranceDeg;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kD;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kHomeAngleDeg;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMaxAngleDeg;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMaxTemperatureCelsius;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMinAngleDeg;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotionMagicAccelerationDegPerSecSq;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotionMagicCruiseVelocityDegPerSec;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotionMagicJerkDegPerSecCubed;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotorCANBus;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotorID;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotorInverted;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMotorSupplyLimitAmps;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kP;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kS;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kStatusSignalFrequencyHz;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kUseFOC;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kV;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Direct Phoenix 6 turret subsystem using a Kraken X44 + Motion Magic.
 *
 * <p>The mechanism is treated as a limited-rotation turret, not a continuous mechanism. The
 * Talon is zeroed to the turret's startup position and both software and software-side clamping
 * keep commands within [-180 deg, +180 deg] to prevent cable wrapping.
 */
public class TurretSubsystem extends SubsystemBase {
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
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
    private final Alert motorDisconnectedAlert =
            new Alert("Turret motor disconnected! (ID: " + kMotorID + ")", AlertType.kError);
    private final Alert motorOverheatAlert =
            new Alert("Turret motor overheating! (ID: " + kMotorID + ")", AlertType.kWarning);

    private double measuredAngleDeg = kHomeAngleDeg;
    private double measuredVelocityDegPerSec = 0.0;
    private double appliedVolts = 0.0;
    private double supplyCurrentAmps = 0.0;
    private double statorCurrentAmps = 0.0;
    private double temperatureCelsius = 0.0;

    private double requestedSetpointDeg = kHomeAngleDeg;
    private double clampedSetpointDeg = kHomeAngleDeg;
    private boolean motorConnected = false;
    private boolean encodersZeroed = false;
    private ControlMode controlMode = ControlMode.NEUTRAL;

    public TurretSubsystem() {
        setName("Turret");
        hardwareEnabled = Constants.currentMode == Constants.Mode.REAL;

        if (hardwareEnabled) {
            motor = kMotorCANBus.isEmpty() ? new TalonFX(kMotorID) : new TalonFX(kMotorID, kMotorCANBus);
            configureMotor();

            motorVoltageSignal = motor.getMotorVoltage();
            velocitySignal = motor.getVelocity();
            positionSignal = motor.getPosition();
            supplyCurrentSignal = motor.getSupplyCurrent();
            statorCurrentSignal = motor.getStatorCurrent();
            temperatureSignal = motor.getDeviceTemp();

            BaseStatusSignal.setUpdateFrequencyForAll(
                    kStatusSignalFrequencyHz,
                    motorVoltageSignal,
                    velocitySignal,
                    positionSignal,
                    supplyCurrentSignal,
                    statorCurrentSignal,
                    temperatureSignal);
            ParentDevice.optimizeBusUtilizationForAll(motor);

            zeroEncoders();
            stop();
            refreshSignals();
        } else {
            motor = null;
            motorVoltageSignal = null;
            velocitySignal = null;
            positionSignal = null;
            supplyCurrentSignal = null;
            statorCurrentSignal = null;
            temperatureSignal = null;
        }
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

        // Per CTRE Phoenix 6 docs, SensorToMechanismRatio lets the Talon operate directly in
        // mechanism units for position/velocity based control.
        config.Feedback.SensorToMechanismRatio = kGearboxReduction;

        // These gains are specified in volts per degree and volts per degree/sec, so convert them
        // to Phoenix's mechanism-rotations units.
        config.Slot0.kP = kP * 360.0;
        config.Slot0.kD = kD * 360.0;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV * 360.0;

        // Motion Magic targets a final position with an onboard trapezoidal motion profile.
        config.MotionMagic.MotionMagicCruiseVelocity =
                kMotionMagicCruiseVelocityDegPerSec / 360.0;
        config.MotionMagic.MotionMagicAcceleration =
                kMotionMagicAccelerationDegPerSecSq / 360.0;
        config.MotionMagic.MotionMagicJerk =
                kMotionMagicJerkDegPerSecCubed / 360.0;

        // The turret is cable-limited, so continuous wrap must stay off.
        config.ClosedLoopGeneral.ContinuousWrap = false;

        // Per CTRE docs, software limits provide controller-side enforcement of the legal travel.
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
            requestedSetpointDeg = getAngle();
            clampedSetpointDeg = requestedSetpointDeg;
            stop();
        }

        motorDisconnectedAlert.set(hardwareEnabled && !motorConnected);
        motorOverheatAlert.set(hardwareEnabled && temperatureCelsius >= kMaxTemperatureCelsius);

        Logger.recordOutput("Turret/HardwareEnabled", hardwareEnabled);
        Logger.recordOutput("Turret/MotorConnected", motorConnected);
        Logger.recordOutput("Turret/EncodersZeroed", encodersZeroed);
        Logger.recordOutput("Turret/ControlMode", controlMode);
        Logger.recordOutput("Turret/RequestedSetpointDeg", requestedSetpointDeg, "degrees");
        Logger.recordOutput("Turret/ClampedSetpointDeg", clampedSetpointDeg, "degrees");
        Logger.recordOutput("Turret/PositionDeg", measuredAngleDeg, "degrees");
        Logger.recordOutput("Turret/VelocityDegPerSec", measuredVelocityDegPerSec, "degrees_per_second");
        Logger.recordOutput("Turret/PositionErrorDeg", clampedSetpointDeg - measuredAngleDeg, "degrees");
        Logger.recordOutput("Turret/AppliedVolts", appliedVolts, "volts");
        Logger.recordOutput("Turret/SupplyCurrentAmps", supplyCurrentAmps, "amps");
        Logger.recordOutput("Turret/StatorCurrentAmps", statorCurrentAmps, "amps");
        Logger.recordOutput("Turret/TemperatureCelsius", temperatureCelsius, "celsius");
        Logger.recordOutput("Turret/NearSetpoint", isNearSetpoint());
        Logger.recordOutput("Turret/AtForwardLimit", measuredAngleDeg >= kMaxAngleDeg - 1.0);
        Logger.recordOutput("Turret/AtReverseLimit", measuredAngleDeg <= kMinAngleDeg + 1.0);
    }

    private void refreshSignals() {
        motorConnected = connectedDebouncer.calculate(BaseStatusSignal.refreshAll(
                        motorVoltageSignal,
                        velocitySignal,
                        positionSignal,
                        supplyCurrentSignal,
                        statorCurrentSignal,
                        temperatureSignal)
                .isOK());

        measuredAngleDeg = Units.rotationsToDegrees(positionSignal.getValueAsDouble());
        measuredVelocityDegPerSec = Units.rotationsToDegrees(velocitySignal.getValueAsDouble());
        appliedVolts = motorVoltageSignal.getValueAsDouble();
        supplyCurrentAmps = Math.abs(supplyCurrentSignal.getValueAsDouble());
        statorCurrentAmps = Math.abs(statorCurrentSignal.getValueAsDouble());
        temperatureCelsius = temperatureSignal.getValueAsDouble();
    }

    /** Commands a turret angle in degrees relative to the startup home position. */
    public void setTurretAngle(double angleDeg) {
        requestedSetpointDeg = angleDeg;
        clampedSetpointDeg = MathUtil.clamp(angleDeg, kMinAngleDeg, kMaxAngleDeg);
        controlMode = ControlMode.MOTION_MAGIC;

        if (hardwareEnabled) {
            motor.setControl(
                    motionMagicRequest.withPosition(Units.degreesToRotations(clampedSetpointDeg)));
        }
    }

    /** Runs the turret in open-loop voltage control. */
    public void setVoltage(double volts) {
        controlMode = ControlMode.VOLTAGE;
        requestedSetpointDeg = getAngle();
        clampedSetpointDeg = requestedSetpointDeg;

        if (hardwareEnabled) {
            motor.setControl(voltageRequest.withOutput(volts));
        }
    }

    /** Stops the turret motor. */
    public void stop() {
        controlMode = ControlMode.NEUTRAL;
        if (hardwareEnabled) {
            motor.setControl(neutralRequest);
        }
    }

    /** Returns the measured turret angle in degrees relative to startup home. */
    public double getAngle() {
        return MathUtil.clamp(measuredAngleDeg, kMinAngleDeg, kMaxAngleDeg);
    }

    /** Returns true when the turret is within tolerance of the commanded setpoint. */
    public boolean isNearSetpoint() {
        return Math.abs(clampedSetpointDeg - getAngle()) <= kAngleToleranceDeg;
    }

    /** Returns the last commanded closed-loop setpoint in degrees. */
    public double getLatestSetpoint() {
        return clampedSetpointDeg;
    }

    /** Zeros the turret to its current startup position so all travel is relative to home. */
    public void zeroEncoders() {
        if (hardwareEnabled) {
            motor.setPosition(Units.degreesToRotations(kHomeAngleDeg));
        }
        measuredAngleDeg = kHomeAngleDeg;
        measuredVelocityDegPerSec = 0.0;
        requestedSetpointDeg = kHomeAngleDeg;
        clampedSetpointDeg = kHomeAngleDeg;
        encodersZeroed = true;
    }

    /** Returns whether the subsystem has established its startup zero reference. */
    public boolean areEncodersZeroed() {
        return encodersZeroed;
    }
}
