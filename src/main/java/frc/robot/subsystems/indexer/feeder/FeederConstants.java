package frc.robot.subsystems.indexer.feeder;

import edu.wpi.first.math.system.plant.DCMotor; //TODO CHANGE FEEDER CONSTANTS

/**
 * The constants required for the feeder subsystem.
 */
public class FeederConstants {
	// Motor IDs
	public static final int kMotorID = 12;
	public static final String kMotorCANBus = "";

	// Control parameters
	// Voltage when intaking, ~2500 RPM for the Kraken X60.
	public static final double kFeedVoltage = 4.5;
	public static final double kUnfeedVoltage = -3.5;

	// Gearbox reduction
	public static final double kGearboxReduction = 0.75; // A 3:4 ratio (I/O)

	// Current limit for the motor
	public static final double kMotorSupplyLimitAmps = 30;
	public static final double kMotorStatorLimitAmps = 45;

    public static final double kPeakForwardDutyCycle = 0.6;
    public static final double kPeakReverseDutyCycle = 0.6;

	public static final boolean kIsFOC = true;

	// Inversions
	public static final boolean kMotorInverted = false;

	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getKrakenX60Foc(1);
	public static final double kMomentOfInertia = 0.000175;
}
