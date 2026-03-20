package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;

// TODO

/**
 * The constants required for the hood subsystem.
 */
public class HoodConstants {
	// TODO(new robot): Verify hood motor CAN ID and CAN bus assignment.
	// Motor IDs
	public static final int kMotorID = 14;
	public static final String kMotorCANBus = ""; // Main RIO bus

	// TODO(new robot): Re-measure hood mechanical range and retune hood closed-loop gains.
	// Control constants (PID-SVA)
	// Units: Volts, Degrees, deg/s
	public static final double kMinAngleDeg = 0, kMaxAngleDeg = 90;
	// M_arm = 0.953kg
	// Voltage gains
	// R: Resistance, kT: Torque coeff., M: Arm mass, r_com: CG distance (meters), G: Reduction
	// I = T / G*kT, I = M*g*r_com / G*kT, V = IR @ w=0,
	// Hence kG at horizontal is approx. R*M*r_com*g / G*kT, V(theta) = kG*cos(theta)
	public static final double kP = 0.08, kD = 0.0009, kS = 0, kV = 0.034, kG = 0.240;
	// public static final double kPSim = 0.18, kDSim = 0.006, kSSim = 0, kVSim = 0, kGSim =
	// 0.3641;
	public static final double kPSim = 0.08, kDSim = 0.0009, kSSim = 0, kVSim = 0.054, kGSim = 0.240;
	// TODO(new robot): Retune Motion Magic velocity and acceleration for the new hood.
	// MotionMagic values
	public static final double kMotionMagicMaxVelocityDegPerSec = 720 * 3.0;
	public static final double kMotionMagicAccelerationDegPerSecSq = 1440 * 3.0;
	public static final boolean kIsFOC = true;

	// TODO(new robot): Revisit thermal limits if the cooling setup differs.
	public static final double kMaxTemperature = 80;

	// TODO(new robot): Re-measure the hood exit-angle offset used by shot calculations.
	public static final double kExitAngleOffset = 80;

	// TODO(new robot): Retune SysId limits if you plan to re-run hood characterization.
	public static final double kSysIdVoltageRampRate = 3;
	public static final double kSysIdVoltageStep = 0.9;
	public static final double kSysIdTimeout = 0.4;

	// TODO(new robot): Set the real mechanical zero/calibration angle for the hood.
	// The angle at which the hood rests at the start of the match
	// Recommended to set to 0 if possible, the minimum angle otherwise.
	public static final double kHoodCalibrationAngle = kMinAngleDeg;

	// TODO(new robot): Verify hood gearbox reduction on the new robot.
	// Gearbox reduction
	public static final double kGearboxReduction = 300.0 / 24.0; // A 4:1 ratio (I/O)

	// TODO(new robot): Revisit current limits if the electrical package differs.
	// Current limits for the motors
	public static final double kMotorSupplyLimitAmps = 50.0;
	public static final double kMotorStatorLimitAmps = 90.0;

	// TODO(new robot): Verify hood motor direction on the new robot.
	// Inversions
	public static final boolean kMotorInverted = false;

	// Sim-only values below; ignore for the cloned real robot unless you care about simulation fidelity.
	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getKrakenX44Foc(1);
	public static final double kCenterOfMassDistance = 0.169; // meters
	public static final double kMomentOfInertia = 0.031090146;
	public static final Transform3d kHoodCentralPivotRobotRelative = Transform3d.kZero;
}
