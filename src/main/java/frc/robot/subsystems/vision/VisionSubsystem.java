// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public VisionSubsystem(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert("Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    /** Returns the observation timestamp for the latest simple target sample, or NaN if unavailable. */
    public double getTargetObservationTimestampSeconds(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservationTimestampSeconds;
    }

    /** Returns the total observation latency for the latest simple target sample. */
    public double getTargetObservationLatencySeconds(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservationLatencySeconds;
    }

    /** Returns the fiducial ID of the latest simple target observation, or -1 if unavailable. */
    public int getTargetTagId(int cameraIndex) {
        return inputs[cameraIndex].latestTargetTagId;
    }

    /** Returns the target area of the latest simple target observation. */
    public double getTargetArea(int cameraIndex) {
        return inputs[cameraIndex].latestTargetArea;
    }

    /**
     * Returns whether the requested camera currently has a valid simple target.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public boolean hasTarget(int cameraIndex) {
        return inputs[cameraIndex].hasTarget;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
            boolean poseEstimationEnabled = isCameraPoseEstimationEnabled(cameraIndex);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                double cameraMaxAmbiguity = getCameraMaxAmbiguity(cameraIndex);
                double cameraMaxZError = getCameraMaxZError(cameraIndex);

                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > cameraMaxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > cameraMaxZError // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                linearStdDev *= getCameraLinearStdDevFactor(cameraIndex);
                angularStdDev *= getCameraAngularStdDevFactor(cameraIndex);

                if (!poseEstimationEnabled) {
                    continue;
                }

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/MaxAmbiguity",
                    getCameraMaxAmbiguity(cameraIndex));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/MaxZError",
                    getCameraMaxZError(cameraIndex));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/LinearStdDevFactor",
                    getCameraLinearStdDevFactor(cameraIndex));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/AngularStdDevFactor",
                    getCameraAngularStdDevFactor(cameraIndex));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/PoseEstimationEnabled",
                    poseEstimationEnabled);
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/HasTarget",
                    inputs[cameraIndex].hasTarget);
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TargetTagId",
                    inputs[cameraIndex].latestTargetTagId);
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TargetArea",
                    inputs[cameraIndex].latestTargetArea);
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TargetXDegrees",
                    inputs[cameraIndex].latestTargetObservation.tx().getDegrees());
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TargetYDegrees",
                    inputs[cameraIndex].latestTargetObservation.ty().getDegrees());
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TargetObservationTimestampSeconds",
                    inputs[cameraIndex].latestTargetObservationTimestampSeconds);
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TargetObservationLatencySeconds",
                    inputs[cameraIndex].latestTargetObservationLatencySeconds);
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/PublishedRobotYawDegrees",
                    inputs[cameraIndex].publishedRobotYawDegrees);
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/PublishedCameraPoseRobotSpace",
                    inputs[cameraIndex].publishedCameraPoseRobotSpace);
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/MegaTag2RejectedForYawRate",
                    inputs[cameraIndex].megatag2RejectedForYawRate);
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/MegaTag2RejectedForUntrustedOrientation",
                    inputs[cameraIndex].megatag2RejectedForUntrustedOrientation);
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
