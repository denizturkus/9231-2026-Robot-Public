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

    // Keep this synchronized with ShootOnTheMoveConstants.kRobotToTurret.
  	public static Transform3d robotToTurret =
    	  new Transform3d(
        	-0.161,
          	0,
          	0.41575,
          	new Rotation3d(0.0, 0.0, Math.PI));

	  public static final double loopPeriodSecs = 0.01;


	// Standard 50 Hz
	public static final double kLoopPeriodSeconds = 1.0 / 50.0;

	public static final class ShootOnTheMoveConstants {
		// TODO: Retune moving-shot latency compensation and filtering
		public static final double kPhaseDelaySeconds = 0.03;
		public static final double kVelocityFilterWindowSeconds = 0.1;
		public static final int kLookaheadIterations = 10;
		public static final double kTurretVisionTxFilterTimeConstantSeconds = 0.08;
		public static final double kTurretVisionTxDeadbandDegrees = 0.35;

		// TODO: Zero this against the turret Limelight crosshair and shooter centerline.
		public static final double kTurretVisionAimOffsetDegrees = 0.0;

		// Keep this synchronized with Constants.robotToTurret.
		public static final Translation2d kRobotToTurret =
				new Translation2d(-0.161,0);

		// TODO: INTERPOLASYON TABLOSUU
		public static final double[][] kHoodAngleDegreesTable =
				new double[][] {
					{1.2, 12},
					{1.5, 17}
				};

		public static final InterpolatingDoubleTreeMap kHoodAngleDegrees =
				createInterpolatingMap(kHoodAngleDegreesTable);

		public static final double[][] kFlywheelSpeedRpmTable =
				new double[][] {
					//{2.5, 4300},
					//{3, 4300},
					{3.5, 4000}
					//{4, 4800}
				};

		public static final InterpolatingDoubleTreeMap kFlywheelSpeedRpm =
				createInterpolatingMap(kFlywheelSpeedRpmTable);

		public static final double[][] kTimeOfFlightSecondsTable =
				new double[][] {
					{1.38, 0.90},
					{1.88, 1.09},
					{3.15, 1.11},
					{4.55, 1.12},
					{5.68, 1.16}
				};

		public static final InterpolatingDoubleTreeMap kTimeOfFlightSeconds =
				createInterpolatingMap(kTimeOfFlightSecondsTable);

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
		// TODO: Re-measure robot footprint, mass, and wheel/floor coefficient
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

    public static DriverStation.Alliance getActiveAlliance() {
        if (currentMode != Mode.REAL) {
            return RobotContainer.currentAlliance;
        }
        return DriverStation.getAlliance().orElse(RobotContainer.currentAlliance);
    }

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
			return getActiveAlliance() == DriverStation.Alliance.Red
				? RED_HUB.value
				: BLUE_HUB.value;
			}

			public static final Translation3d getAllianceOutpostPosition() {
			return getActiveAlliance() == DriverStation.Alliance.Red
				? RED_OUTPOST.value
				: BLUE_OUTPOST.value;
			}

			public static final Translation3d getAllianceFarSidePosition() {
			return getActiveAlliance() == DriverStation.Alliance.Red
				? RED_FAR_SIDE.value
				: BLUE_FAR_SIDE.value;
			}

			public static final Translation3d getAllianceSideTargetPosition(
					Translation2d currentRobotPosition) {
				Translation3d outpost = getAllianceOutpostPosition();
				Translation3d farSide = getAllianceFarSidePosition();
				if (currentRobotPosition == null) {
					return outpost;
				}

				return currentRobotPosition.getDistance(outpost.toTranslation2d())
								<= currentRobotPosition.getDistance(farSide.toTranslation2d())
						? outpost
						: farSide;
			}
		}
		}

}
