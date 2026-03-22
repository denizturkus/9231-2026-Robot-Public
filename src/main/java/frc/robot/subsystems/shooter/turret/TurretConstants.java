package frc.robot.subsystems.shooter.turret;

/** Constants for the direct-drive turret subsystem. */
public final class TurretConstants {
    private TurretConstants() {}

    public static final int kMotorID = 13;
    public static final String kMotorCANBus = ""; // Main RIO CAN bus

    // Kraken X44 on a 16:200 turret reduction -> 12.5:1 sensor-to-mechanism ratio.
    public static final double kGearboxReduction = 200.0 / 16.0;

    // The turret reports 0 deg at its startup home position after zeroEncoders() is called.
    public static final double kHomeAngleDeg = 0.0;
    public static final double kMinAngleDeg = -180.0;
    public static final double kMaxAngleDeg = 180.0;
    // TODO: Tighten this back down after turret tuning is finished.
    // Previous value for tuned-ready checks:
    // public static final double kAngleToleranceDeg = 5.0;
    public static final double kAngleToleranceDeg = 8.0;

    // Phoenix closed-loop gains. These are still tuning values, but Motion Magic handles the
    // profile generation and onboard PID.
    public static final double kP = 0.24;
    public static final double kD = 0.0050;
    public static final double kS = 0.0;
    public static final double kV = 0.0078;

    public static final double kMotionMagicCruiseVelocityDegPerSec = 700.0;
    public static final double kMotionMagicAccelerationDegPerSecSq = 1250.0;
    public static final double kMotionMagicJerkDegPerSecCubed = 0.0;


    public static final boolean kUseFOC = true; // Phoenix Pro feature
    public static final boolean kMotorInverted = true;
	
    public static final double kMotorSupplyLimitAmps = 60.0;
    public static final double kMotorStatorLimitAmps = 120.0;
    public static final double kMaxTemperatureCelsius = 80.0;

    public static final double kStatusSignalFrequencyHz = 100.0;
}
