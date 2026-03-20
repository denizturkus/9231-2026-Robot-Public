package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The constants required for the turret subsystem.
 */
public class TurretConstants {
	// TODO(new robot): Verify turret motor CAN ID and CAN bus assignment.
	// Motor IDs
	public static final int kMotorID = 13;
	public static final String kMotorCANBus = ""; // Main RIO bus

	// TODO(new robot): Re-measure turret range limits and retune turret closed-loop gains.
	// NOTE: All angles are bound within (-180, 180)
	// Control constants (PID-SVA)
	// Units: Volts, Degrees, deg/s
	// Limits due to cable wrapping, hard-stop limits. If cable wrapping is the
	// issue,
	// These angles may have values with magnitude higher then 180
	// The min angle should be negative, and the max angle should be positive
	public static final double kMinAngleDeg = -90, kMaxAngleDeg = 270;
	// Voltage gains, feedforward gains
	public static final double kP = 0.19, kD = 0.0031, kS = 0, kV = 0.008087;
	public static final double kPSim = 0.19, kDSim = 0.0031, kSSim = 0, kVSim = 0.008087;
	// public static final double kPSim = 0.11/* 0.315 * 0.8 */, kDSim = 0.004,p
	// kSSim = 0, kVSim = 0.0026982, kASim = 4.1409E-05; MotionMagic values
	// TODO(new robot): Retune Motion Magic velocity and acceleration for the new turret.
	public static final double kMotionMagicMaxVelocityDegPerSec = 2000;
	public static final double kMotionMagicAccelerationDegPerSecSq = 7000;
	public static final boolean kIsFOC = true;

	// TODO(new robot): Revisit thermal limits if the cooling setup differs.
	public static final double kMaxTemperature = 80;

	// TODO(new robot): Verify deadzone lookup range if hard-stops or cable management differ.
	public static final int kDeadzoneLookupRange = 2;

	// TODO(new robot): Retune SysId limits if you plan to re-run turret characterization.
	public static final double kSysIdVoltageRampRate = 0.1;
	public static final double kSysIdVoltageStep = 0.6;
	public static final double kSysIdTimeout = 3;

	// TODO(new robot): Set the real turret zero/calibration angle for the new robot.
	// TODO 0 or 180, needs testing on real robot
	// The angle at which the turret rests at the start of the match
	public static final double kTurretCalibrationAngle = 180.0;

	// TODO(new robot): Verify turret gearbox reduction on the new robot.
	// Gearbox reduction
	public static final double kGearboxReduction = 30; // A 10:1 ratio

	// TODO(new robot): Revisit current limits if the electrical package differs.
	// Current limits for the motors
	public static final double kMotorSupplyLimitAmps = 60.0;
	public static final double kMotorStatorLimitAmps = 120.0;

	// TODO(new robot): Verify turret motor direction on the new robot.
	// Inversions
	public static final boolean kMotorInverted = false;

	// Sim-only values below; ignore for the cloned real robot unless you care about simulation fidelity.
	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getKrakenX44Foc(1);
	public static final double kMomentOfInertia = 0.0388802898;
	public static final Pose3d kTurretPivotPointCenter = Pose3d.kZero;
}
