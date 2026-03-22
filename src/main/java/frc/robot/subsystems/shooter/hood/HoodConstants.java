package frc.robot.subsystems.shooter.hood;

/** Constants for the direct-drive hood subsystem. */
public final class HoodConstants {
	private HoodConstants() {}

	public static final int kMotorID = 14;
	public static final String kMotorCANBus = ""; // Main RIO CAN bus

	// Mechanism reports degrees directly through SensorToMechanismRatio.
	public static final double kGearboxReduction = 405.0 / 28.0;

	public static final double kMinAngleDeg = 0;
	public static final double kHomeAngleDeg = kMinAngleDeg;
	public static final double kMaxAngleDeg = 32; //soft limit for now

	// Previous value for tuned-ready checks:
	public static final double kAngleToleranceDeg = 2.0;
	// Phoenix closed-loop gains in volts per degree and volts per degree/sec.
	public static final double kP = 0.36;
	public static final double kD = 0.003;
	public static final double kS = 0.0;
	public static final double kV = 0.01;
	public static final double kG = 0.325;

	public static final double kMotionMagicCruiseVelocityDegPerSec = 420.0;
	public static final double kMotionMagicAccelerationDegPerSecSq = 1200.0;

	public static final boolean kUseFOC = true;
	public static final boolean kMotorInverted = false;

	public static final double kMotorSupplyLimitAmps = 50.0;
	public static final double kMotorStatorLimitAmps = 90.0;
	public static final double kMaxTemperatureCelsius = 80.0;
	public static final double kStatusSignalFrequencyHz = 100.0;

	public static final double kSysIdVoltageRampRate = 3.0;
	public static final double kSysIdVoltageStep = 0.9;
	public static final double kSysIdTimeout = 0.4;
}
