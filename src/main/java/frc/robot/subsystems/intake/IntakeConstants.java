package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor; //TODO Change Intake Constants

/**
 * The constants required for the intake subsystem.
 */
public class IntakeConstants {
	// Motor IDs
	public static final int kRollerMotorID = 9;
	public static final String kRollerMotorCANBus = ""; // Main RIO bus
	public static final int kArmMotorID = 10;
	public static final String kArmMotorCANBus = "";

	// Control parameters
	public static final boolean kIsFOC = true;
	// Voltage when intaking, ~2000-2500 RPM for the Kraken X60.
	public static final double kIntakeRollerVoltage = 5;
	public static final double kOuttakeRollerVoltage = -3.5;
	// Arm PID/feedforward
	// Units: Volts, Degrees
	// Keep enough authority to overcome gravity and stiction near the endpoints
	// without making the arm motion feel too abrupt.
	public static final double kS = 1.2;
	public static final double kP = 0.18, kD = 0, kG = 1.0;
	public static final double kSSim = 1.2;
	public static final double kPSim = 0.18, kDSim = 0, kGSim = 1.0;
	public static final double kMotionMagicMaxVelocityDegPerSec = 77;
	public static final double kMotionMagicAccelerationDegPerSecSq = 150;
	public static final double kRetractMotionMagicMaxVelocityDegPerSec = 100;
	public static final double kRetractMotionMagicAccelerationDegPerSecSq = 225;
	// The angle at which the encoders are zeroed, the angle at the start of the match
	public static final double kArmCalibrationAngleDeg = 145;
	public static final double kArmClosedAngle = 145, kArmOpenedAngle = 0;
	public static final double kArmMinAngle = 0, kArmMaxAngle = 145;

	public static final double kArmMaxTemperature = 80;
	public static final double kRollerMaxTemperature = 80;

	// Gearbox reduction
	public static final double kRollerGearboxReduction = 1; // A 1:1 ratio (I/O)
	public static final double kArmGearboxReduction = 60 * 15 / 17; //

	// Current limits for the motor
	public static final double kRollerMotorSupplyLimitAmps = 20.0;
	public static final double kRollerMotorStatorLimitAmps = 35.0;

	public static final double kArmMotorSupplyLimitAmps = 30.0;
	public static final double kArmMotorStatorLimitAmps = 40.0;

	// Inversions
	public static final boolean kRollerMotorInverted = false;
	public static final boolean kArmMotorInverted = false;

	// Sim-only values below; ignore for the cloned real robot unless you care about simulation fidelity.
	// Simulation data
	// Roller simulation
	public static final DCMotor kRollerGearbox = DCMotor.getKrakenX60Foc(1);
	public static final double kRollerMomentOfInertia = 0.000175;
	// Arm simulation
	public static final DCMotor kArmGearbox = DCMotor.getKrakenX60Foc(1);
	public static final double kArmCenterOfGravityDistance = 0.12; // meters
	// MOI of the arm through the pivot axis, unit kg*m^2
	public static final double kArmMomentOfInertiaPivot = 0.0016;
	// public static final Translation3d kIntakePivotPoseRobotRelative = Translation3d.kZero;
}
