package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;

// TODO change flywheel constants

/**
 * The constants required for the flywheel subsystem.
 */
public class FlywheelConstants {
	// Motor IDs
	public static final int kLeadMotorID = 20;
    public static final int kFollowerMotorID = 21;

	public static final String kMotorCANBus = ""; // Main RIO bus

	// Control constants (PID-SVA)
	// Units: Volts, RPM
	public static final double kP = 0.0104, kD = 5.28e-5, kS = 0.0, kV = 0.0015448;
	public static final double kPSim = 0.0104, kDSim = 5.28e-5, kSSim = 0.0, kVSim = 0.0015448;
	public static final boolean kIsFOC = true;

	public static final double kMaxTemperature = 80;

	public static final double kSysIdVoltageRampRate = 0.6;
	public static final double kSysIdVoltageStep = 3;
	public static final double kSysIdTimeout = 5;

	// Soft-max, not the hardware limit RPM
	public static final double kMaxAllowedRPM = 3000;

	// Gearbox reduction
	public static final double kGearboxReduction = 1; // A 1:1 ratio (I/O)

	// Current limits for the motors
	public static final double kMotorSupplyLimitAmps = 50.0;
	public static final double kMotorStatorLimitAmps = 90.0;

	// Inversions
	public static final boolean kLeadMotorInverted = false;
    public static final boolean kFollowerOpposesLeader = true;

	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getKrakenX60Foc(2);
	public static final double kMomentOfInertia = 0.000774324522;
}
