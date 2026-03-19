package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {

    public static final Pose2d kStartingPose = new Pose2d(3, 2, Rotation2d.kZero);

	// Standard 50 Hz
	public static final double kLoopPeriodSeconds = 1.0 / 50.0;

	public static final double kDriverControllerDeadband = 0.05;

	public static final boolean kIsReplay = false;
	public static final boolean kDoTuning = true;

	/** Class holding the physical parameters of the robot. */
	public static final class PhysicalParameters {
		// Robot measurements
		public static final Distance kRobotLength = Centimeters.of(67.2), kRobotWidth = Centimeters.of(67.2);
		public static final Mass kRobotMass = Kilograms.of(50);
		public static final double kWheelCOF = 1.426;

		public static final double kBatteryInternalResistance = 0.016;
		public static final double kNominalVoltage = 12.8;
		public static final double kRobotWarningBatteryVoltageThreshold = 11.6;
	}
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

}