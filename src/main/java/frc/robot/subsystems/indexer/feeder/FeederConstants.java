package frc.robot.subsystems.indexer.feeder;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The constants required for the feeder subsystem.
 */
public class FeederConstants {
	// Motor IDs
	public static final int kMotorID = 12;
	public static final String kMotorCANBus = "";

	// Control parameters
	// Voltage when feeding, ~2000 RPM for the Kraken X60.
	public static final double kFeedVoltage = 9;
	public static final double kUnfeedVoltage = -3.5;

	// Gearbox reduction
	public static final double kGearboxReduction = 4;

	// Current limit for the motor
	public static final double kMotorSupplyLimitAmps = 40;
	public static final double kMotorStatorLimitAmps = 60;

    public static final double kPeakForwardDutyCycle = 0.6;
    public static final double kPeakReverseDutyCycle = 0.6;

	public static final boolean kIsFOC = true;

	// Inversions
	public static final boolean kMotorInverted = true; 

	// Sim-only values below; ignore for the cloned real robot unless you care about simulation fidelity.
	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getKrakenX60Foc(1);
	public static final double kMomentOfInertia = 0.000175;
}
