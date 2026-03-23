package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout; 
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

public class VisionConstants {
    public static final int chassisCameraIndex = 0;
    public static final int turretCameraIndex = 1;

    // 2026 REBUILT Game Manual 5.11: HUB AprilTag IDs on the welded field.
    public static final int[] hubAprilTagIds =
            new int[] {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27};

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);


    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight-chassis";
    public static String camera1Name = "limelight-turret";

    //TODO: re-measure turretToCamera1 from CAD
    // Robot to camera transforms
    // `turrettToCamera1` is the turret camera transform when the turret is at its zero angle.
    public static Transform3d robotToCamera0 = new Transform3d(-0.27248, 0.2176, 0.14083, new Rotation3d(0.0, (Math.PI / 6.0) , (7 * Math.PI) / 4.0));
    public static Transform3d turretToCamera1 = new Transform3d(-0.151, 0.0, 0, new Rotation3d(0.0, 0.0, Math.PI));

    // TODO: Retune global vision rejection thresholds after pose validation
    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // TODO: Retune per-camera rejection thresholds
    // Per-camera rejection thresholds.
    // The turret camera is slightly stricter because its transform changes with turret motion.
    public static double[] cameraMaxAmbiguities = new double[] {
        0.3, // Chassis camera
        0.25 // Turret camera
    };
    public static double[] cameraMaxZErrors = new double[] {
        0.75, // Chassis camera
        0.6 // Turret camera
    };

    // TODO: Confirm which cameras should contribute pose updates
    // The chassis camera is the dedicated odometry camera. The turret camera is reserved for targeting.
    public static boolean[] cameraPoseEstimationEnabled = new boolean[] {
        true, // Chassis camera
        false // Turret camera
    };

    // TODO: Retune global vision trust baselines after validating pose error
    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // TODO: Retune per-camera trust multipliers for the chassis and turret Limelights.
    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraLinearStdDevFactors = new double[] {
        1.0, // Chassis camera
        1.0 // Turret camera
    };
    public static double[] cameraAngularStdDevFactors = new double[] {
        1.0, // Chassis camera
        1.75 // Turret camera
    };

    // TODO: Revisit MegaTag 2 scaling 
    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

    public static double getCameraMaxAmbiguity(int cameraIndex) {
        return getCameraValue(cameraMaxAmbiguities, cameraIndex, maxAmbiguity);
    }

    public static double getCameraMaxZError(int cameraIndex) {
        return getCameraValue(cameraMaxZErrors, cameraIndex, maxZError);
    }

    public static double getCameraLinearStdDevFactor(int cameraIndex) {
        return getCameraValue(cameraLinearStdDevFactors, cameraIndex, 1.0);
    }

    public static double getCameraAngularStdDevFactor(int cameraIndex) {
        return getCameraValue(cameraAngularStdDevFactors, cameraIndex, 1.0);
    }

    public static boolean isCameraPoseEstimationEnabled(int cameraIndex) {
        return getCameraValue(cameraPoseEstimationEnabled, cameraIndex, true);
    }

    public static int[] getSimpleTargetAllowedTagIds(int cameraIndex) {
        return switch (cameraIndex) {
            case turretCameraIndex -> hubAprilTagIds;
            default -> new int[0];
        };
    }

    public static Transform3d getTurretCameraTransform(Rotation2d turretAngle) {
        Pose3d robotOrigin = new Pose3d();
        Pose3d turretPivotPose = robotOrigin.plus(Constants.robotToTurret);
        Pose3d zeroAngleCameraPose = robotOrigin.plus(turretToCamera1);
        Transform3d turretToCameraAtZero = new Transform3d(turretPivotPose, zeroAngleCameraPose);
        Transform3d turretRotation =
                new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, turretAngle.getRadians()));
        Pose3d currentCameraPose = turretPivotPose.plus(turretRotation).plus(turretToCameraAtZero);
        return new Transform3d(robotOrigin, currentCameraPose);
    }

    private static double getCameraValue(double[] values, int cameraIndex, double defaultValue) {
        return cameraIndex >= 0 && cameraIndex < values.length ? values[cameraIndex] : defaultValue;
    }

    private static boolean getCameraValue(boolean[] values, int cameraIndex, boolean defaultValue) {
        return cameraIndex >= 0 && cameraIndex < values.length ? values[cameraIndex] : defaultValue;
    }
}
