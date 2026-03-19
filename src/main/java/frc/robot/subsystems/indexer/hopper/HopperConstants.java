package frc.robot.subsystems.indexer.hopper;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Hopper subsystem constants.
 */
public final class HopperConstants {
    private HopperConstants() {}

    public static final int kMotorId = 11;
    public static final String kCANBus = "";

    // Motor config
    public static final InvertedValue kMotorInverted = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    // Current limits
    public static final double kSupplyCurrentLimitAmps = 35.0;
    public static final double kStatorCurrentLimitAmps = 60.0;
    public static final boolean kEnableSupplyCurrentLimit = true;
    public static final boolean kEnableStatorCurrentLimit = true;

    // Open-loop voltaj komutları
    public static final double kFeedVoltage = 6.0;       // feed
    public static final double kFastFeedVoltage = 10.0;  // fastfeed
    public static final double kReverseVoltage = -4.0;   // unjam 
    public static final double kStopVoltage = 0.0;

    // Telemetry / logging
    public static final double kStatusSignalFrequencyHz = 50.0;

    // protection
    public static final double kJamCurrentThresholdAmps = 50.0;
    public static final double kHighTempThresholdCelsius = 80.0;
}