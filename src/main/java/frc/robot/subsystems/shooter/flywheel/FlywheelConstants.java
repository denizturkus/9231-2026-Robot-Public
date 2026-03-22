package frc.robot.subsystems.shooter.flywheel;

/** Constants for the direct-drive flywheel subsystem. */
public final class FlywheelConstants {
    private FlywheelConstants() {}

    public static final int kLeadMotorID = 21;
    public static final int kFollowerMotorID = 22;
    public static final String kMotorCANBus = ""; // Main RIO CAN bus

    // Mechanism velocity is reported directly in flywheel units through SensorToMechanismRatio.
    public static final double kGearboxReduction = 15.0 / 33.0;

    // Phoenix closed-loop gains in volts per RPM and volts per RPM/sec.
    public static final double kP = 0.003; 
    public static final double kD = 0.0;
    public static final double kS = 0.52557;
    public static final double kV = 0.058992 / 60.0; //0.00136

    public static final boolean kUseFOC = true;
    public static final boolean kLeadMotorInverted = false;
    public static final boolean kFollowerOpposesLeader = true;

    public static final double kMotorSupplyLimitAmps = 70.0;
    public static final double kMotorStatorLimitAmps = 120.0;
    public static final double kMaxTemperatureCelsius = 80.0;
    public static final double kStatusSignalFrequencyHz = 100.0;

    // Soft max, not the hardware limit.
    public static final double kMaxAllowedRPM = 6000.0;
    // Require a tighter band to start feeding, then allow a wider hold band so
    // shot-to-shot RPM dips do not chatter the feeder/hopper.
    public static final double kVelocityToleranceRPM = 50.0;
    public static final double kVelocityHysteresisToleranceRPM = 150.0;

    public static final double kSysIdVoltageRampRate = 0.6;
    public static final double kSysIdVoltageStep = 3.0;
    public static final double kSysIdTimeout = 5.0;
}
