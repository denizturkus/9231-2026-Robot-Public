package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor; //TODO Change Intake Constants

/**
 * The constants required for the intake subsystem.
 */
public class IntakeConstants {
	// TODO(new robot): Verify intake roller/arm CAN IDs and CAN bus assignments.
	// Motor IDs
	public static final int kRollerMotorID = 9;
	public static final String kRollerMotorCANBus = ""; // Main RIO bus
	public static final int kArmMotorID = 10;
	public static final String kArmMotorCANBus = "";

	// TODO(new robot): Verify intake roller voltages and retune arm gains on the new robot.
	// Control parameters
	public static final boolean kIsFOC = true;
	// Voltage when intaking, ~2000-2500 RPM for the Kraken X60.
	public static final double kIntakeRollerVoltage = 4.5;
	public static final double kOuttakeRollerVoltage = -3.5;
	// Arm PID
	// Units: Volts, Degrees
	public static final double kP = 0.05, kD = 0, kG = 0;
	public static final double kPSim = 0.05, kDSim = 0, kGSim = 0;
	// TODO(new robot): Retune Motion Magic velocity and acceleration for the intake arm.
	public static final double kMotionMagicMaxVelocityDegPerSec = 720;
	public static final double kMotionMagicAccelerationDegPerSecSq = 2880 * 2;
	// TODO(new robot): Re-measure intake calibration, open, closed, min, and max arm angles.
	// The angle at which the encoders are zeroed, the angle at the start of the match
	public static final double kArmCalibrationAngleDeg = 90;
	public static final double kArmClosedAngle = 90, kArmOpenedAngle = 0;
	public static final double kArmMinAngle = 0, kArmMaxAngle = 90;

	// TODO(new robot): Revisit temperature limits if the cooling setup differs.
	public static final double kArmMaxTemperature = 80;
	public static final double kRollerMaxTemperature = 80;

	// TODO(new robot): Verify roller and arm reductions on the new robot.
	// Gearbox reduction
	public static final double kRollerGearboxReduction = 1; // A 1:1 ratio (I/O)
	public static final double kArmGearboxReduction = 5; // A 5:1 ratio (I/O)

	// TODO(new robot): Revisit current limits if the electrical package differs.
	// Current limits for the motor
	public static final double kRollerMotorSupplyLimitAmps = 20.0;
	public static final double kRollerMotorStatorLimitAmps = 50.0;

	public static final double kArmMotorSupplyLimitAmps = 30.0;
	public static final double kArmMotorStatorLimitAmps = 60.0;

	// TODO(new robot): Verify roller and arm motor directions on the new robot.
	// Inversions
	public static final boolean kRollerMotorInverted = false;
	public static final boolean kArmMotorInverted = true;

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
