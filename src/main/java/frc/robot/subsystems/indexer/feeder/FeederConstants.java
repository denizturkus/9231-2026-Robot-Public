package frc.robot.subsystems.indexer.feeder;

import edu.wpi.first.math.system.plant.DCMotor; //TODO CHANGE FEEDER CONSTANTS

/**
 * The constants required for the feeder subsystem.
 */
public class FeederConstants {
	// TODO: Verify feeder motor CAN ID and CAN bus assignment.
	// Motor IDs
	public static final int kMotorID = 12;
	public static final String kMotorCANBus = "";

	// TODO: Verify feeder feed/unfeed voltages on the new robot.
	// Control parameters
	// Voltage when intaking, ~2500 RPM for the Kraken X60.
	public static final double kFeedVoltage = 6.5;
	public static final double kUnfeedVoltage = -3.5;

	// TODO: Verify feeder gearbox reduction on the new robot.
	// Gearbox reduction
	public static final double kGearboxReduction = 4;

	// TODO: Revisit current limits if the electrical package differs.
	// Current limit for the motor
	public static final double kMotorSupplyLimitAmps = 30;
	public static final double kMotorStatorLimitAmps = 45;

    // TODO: Verify peak duty-cycle limits on the new robot.
    public static final double kPeakForwardDutyCycle = 0.6;
    public static final double kPeakReverseDutyCycle = 0.6;

	public static final boolean kIsFOC = true;

	// TODO: Verify feeder motor direction on the new robot.
	// Inversions
	public static final boolean kMotorInverted = true; 

	// Sim-only values below; ignore for the cloned real robot unless you care about simulation fidelity.
	// Simulation data
	public static final DCMotor kGearbox = DCMotor.getKrakenX60Foc(1);
	public static final double kMomentOfInertia = 0.000175;
}
