package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;


/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {

    public static final Pose2d kStartingPose = new Pose2d(3, 2, Rotation2d.kZero);

    // TODO(new robot): Measure the turret pivot pose relative to the robot origin on the new robot.
    // Keep this synchronized with ShootOnTheMoveConstants.kRobotToTurret.
  	public static Transform3d robotToTurret =
    	  new Transform3d(
        	Inches.of(-5.25).in(Meters),
          	Inches.of(5.25).in(Meters),
          	Inches.of(16.945).in(Meters),
          	new Rotation3d(0.0, 0.0, Math.PI));

	  public static final double loopPeriodSecs = 0.01;


	// Standard 50 Hz
	public static final double kLoopPeriodSeconds = 1.0 / 50.0;

	public static final class ShootOnTheMoveConstants {
		// TODO(new robot): Retune moving-shot latency compensation and filtering on the real robot.
		public static final double kPhaseDelaySeconds = 0.03;
		public static final double kVelocityFilterWindowSeconds = 0.1;
		public static final int kLookaheadIterations = 10;
		public static final boolean kUseTurretVisionCorrection = true;

		// TODO(new robot): Zero this against the turret Limelight crosshair and shooter centerline.
		public static final double kTurretVisionAimOffsetDegrees = 0.0;

		// TODO(new robot): Revalidate the supported moving-shot distance window for the new robot.
		public static final double kMinDistanceMeters = 1.34;
		public static final double kMaxDistanceMeters = 5.60;

		// TODO(new robot): Measure the turret XY offset from the robot origin.
		// Keep this synchronized with Constants.robotToTurret.
		public static final Translation2d kRobotToTurret =
				new Translation2d(Inches.of(-5.25).in(Meters), Inches.of(5.25).in(Meters));

		// TODO(new robot): Rebuild this hood table from real shot testing for the new robot.
		public static final InterpolatingDoubleTreeMap kHoodAngleDegrees =
				createInterpolatingMap(
						new double[][] {
							{1.34, 19.0},
							{1.78, 19.0},
							{2.17, 24.0},
							{2.81, 27.0},
							{3.82, 29.0},
							{4.09, 30.0},
							{4.40, 31.0},
							{4.77, 32.0},
							{5.57, 32.0},
							{5.60, 35.0}
						});

		// TODO(new robot): Rebuild this flywheel table from real shot testing for the new robot.
		public static final InterpolatingDoubleTreeMap kFlywheelSpeedRpm =
				createInterpolatingMap(
						new double[][] {
							{2.12923, 2725.0},
							{2.504222, 2800.0},
							{2.889, 2950.0},
							{3.254686, 3085.0},
							{3.695324, 3200.0},
							{3.983757, 3320.0},
							{4.498437, 3475.0},
							{4.986071, 3675.0},
							{5.410986, 4000.0}
						});

		// TODO(new robot): Rebuild this time-of-flight table from logged real-shot data for the new robot.
		public static final InterpolatingDoubleTreeMap kTimeOfFlightSeconds =
				createInterpolatingMap(
						new double[][] {
							{1.38, 0.90},
							{1.88, 1.09},
							{3.15, 1.11},
							{4.55, 1.12},
							{5.68, 1.16}
						});

		private static InterpolatingDoubleTreeMap createInterpolatingMap(double[][] points) {
			InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
			for (double[] point : points) {
				map.put(point[0], point[1]);
			}
			return map;
		}
	}

	public static final double kDriverControllerDeadband = 0.05;

	public static final boolean kIsReplay = false;
	public static final boolean kDoTuning = true;

	/** Class holding the physical parameters of the robot. */
	public static final class PhysicalParameters {
		// TODO(new robot): Re-measure robot footprint, mass, and wheel/floor coefficient for the new robot.
		// Robot measurements
		public static final Distance kRobotLength = Centimeters.of(67.2), kRobotWidth = Centimeters.of(67.2);
		public static final Mass kRobotMass = Kilograms.of(50);
		public static final double kWheelCOF = 1.426;

		// TODO(new robot): Update battery values only if the electrical package meaningfully differs.
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

	public static class GenericConstants {
		public static boolean disableHAL = false;

		public static void disableHAL() {
			disableHAL = true;
		}

		public static enum AimPoints {
			RED_HUB(new Translation3d(11.938, 4.034536, 1.5748)),
			RED_OUTPOST(new Translation3d(15.75, 7.25, 0)),
			RED_FAR_SIDE(new Translation3d(15.75, 0.75, 0)),

			BLUE_HUB(new Translation3d(4.5974, 4.034536, 1.5748)),
			BLUE_OUTPOST(new Translation3d(0.75, 0.75, 0)),
			BLUE_FAR_SIDE(new Translation3d(0.75, 7.25, 0));

			public final Translation3d value;

			private AimPoints(Translation3d value) {
			this.value = value;
			}

			public static final Translation3d getAllianceHubPosition() {
			return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
				? RED_HUB.value
				: BLUE_HUB.value;
			}

			public static final Translation3d getAllianceOutpostPosition() {
			return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
				? RED_OUTPOST.value
				: BLUE_OUTPOST.value;
			}

			public static final Translation3d getAllianceFarSidePosition() {
			return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
				? RED_FAR_SIDE.value
				: BLUE_FAR_SIDE.value;
			}
		}
		}

}
