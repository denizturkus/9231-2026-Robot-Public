package frc.robot.subsystems.indexer.hopper;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Hopper subsystem constants.
 */
public final class HopperConstants {
    private HopperConstants() {}

    // TODO(new robot): Verify hopper motor CAN ID and CAN bus assignment.
    public static final int kMotorId = 11;
    public static final String kCANBus = "";

    // TODO(new robot): Verify hopper motor inversion and neutral behavior on the new robot.
    // Motor config
    public static final InvertedValue kMotorInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    // TODO(new robot): Revisit current limits if the electrical package differs.
    // Current limits
    public static final double kSupplyCurrentLimitAmps = 35.0;
    public static final double kStatorCurrentLimitAmps = 60.0;
    public static final boolean kEnableSupplyCurrentLimit = true;
    public static final boolean kEnableStatorCurrentLimit = true;

    // TODO(new robot): Verify hopper feed/reverse voltages on the new robot.
    // Open-loop voltage commands
    public static final double kFeedVoltage = 4.5;
    public static final double kFastFeedVoltage = 10.0;
    public static final double kReverseVoltage = -4.0;
    public static final double kStopVoltage = 0.0;

    // TODO(new robot): Revisit status signal rates only if CAN utilization changes.
    // Telemetry / logging
    public static final double kStatusSignalFrequencyHz = 50.0;

    // TODO(new robot): Revisit protection thresholds if the motor or mechanism differs.
    // protection
    public static final double kJamCurrentThresholdAmps = 50.0;
    public static final double kHighTempThresholdCelsius = 80.0;
}
