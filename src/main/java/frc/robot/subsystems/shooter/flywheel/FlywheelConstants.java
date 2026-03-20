package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;

// TODO change flywheel constants

/**
 * The constants required for the flywheel subsystem.
 */
public class FlywheelConstants {
	// TODO(new robot): Verify flywheel motor CAN IDs and CAN bus assignment.
	// Motor IDs
	public static final int kLeadMotorID = 21;
    public static final int kFollowerMotorID = 22;

	public static final String kMotorCANBus = ""; // Main RIO bus

	// TODO(new robot): Retune flywheel velocity gains on the new robot.
	// Control constants (PID-SVA)
	// Units: Volts, RPM
	public static final double kP = 0.0104, kD = 5.28e-5, kS = 0.0, kV = 0.0015448;
	public static final double kPSim = 0.0104, kDSim = 5.28e-5, kSSim = 0.0, kVSim = 0.0015448;
	public static final boolean kIsFOC = true;

	// TODO(new robot): Revisit thermal limits if the cooling setup differs.
	public static final double kMaxTemperature = 80;

	// TODO(new robot): Retune SysId limits if you plan to re-run flywheel characterization.
	public static final double kSysIdVoltageRampRate = 0.6;
	public static final double kSysIdVoltageStep = 3;
	public static final double kSysIdTimeout = 5;

	// TODO(new robot): Set the true safe flywheel RPM limit for the new robot.
	// Soft-max, not the hardware limit RPM
	public static final double kMaxAllowedRPM = 6000;

	// TODO(new robot): Verify mechanical reduction if the flywheel gearbox differs.
	// Gearbox reduction
	public static final double kGearboxReduction = 1; // A 1:1 ratio (I/O)

	// TODO(new robot): Revisit current limits if the electrical package differs.
	// Current limits for the motors
	public static final double kMotorSupplyLimitAmps = 50.0;
	public static final double kMotorStatorLimitAmps = 90.0;

	// TODO(new robot): Verify motor directions on the new robot.
	// Inversions
	public static final boolean kLeadMotorInverted = false;
    public static final boolean kFollowerOpposesLeader = true;

	// Sim-only values below; ignore for the cloned real robot unless you care about simulation fidelity.
	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getKrakenX60Foc(2);
	public static final double kMomentOfInertia = 0.000774324522;
}
