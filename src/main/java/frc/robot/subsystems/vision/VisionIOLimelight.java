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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.LimelightHelpers;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
    private final String name;
    private final Supplier<Rotation2d> robotRotationSupplier;
    private final DoubleSupplier robotYawRateSupplierDegPerSec;
    private final BooleanSupplier robotOrientationTrustedSupplier;
    private final Supplier<Transform3d> robotToCameraSupplier;

    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber captureLatencySubscriber;
    private final DoubleSubscriber tvSubscriber;
    private final DoubleSubscriber tidSubscriber;
    private final DoubleSubscriber taSubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;
    private final int[] simpleTargetAllowedTagIds;
    private final boolean enableMegaTag1;
    private final boolean enableMegaTag2;
    private final Set<Integer> simpleTargetAllowedTagIdSet = new HashSet<>();
    private boolean simpleTargetFilterConfigured = false;

    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name The configured name of the Limelight.
     * @param robotRotationSupplier Supplier for the current estimated robot rotation, used for MegaTag 2.
     * @param robotToCameraSupplier Supplier for the current camera transform relative to the robot.
     */
    public VisionIOLimelight(
            String name,
            Supplier<Rotation2d> robotRotationSupplier,
            Supplier<Transform3d> robotToCameraSupplier) {
        this(name, robotRotationSupplier, () -> 0.0, () -> true, robotToCameraSupplier, new int[0], true, true);
    }

    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name The configured name of the Limelight.
     * @param robotRotationSupplier Supplier for the current estimated robot rotation, used for MegaTag 2.
     * @param robotYawRateSupplierDegPerSec Supplier for the current robot yaw rate in degrees/sec.
     * @param robotToCameraSupplier Supplier for the current camera transform relative to the robot.
     */
    public VisionIOLimelight(
            String name,
            Supplier<Rotation2d> robotRotationSupplier,
            DoubleSupplier robotYawRateSupplierDegPerSec,
            Supplier<Transform3d> robotToCameraSupplier) {
        this(
                name,
                robotRotationSupplier,
                robotYawRateSupplierDegPerSec,
                () -> true,
                robotToCameraSupplier,
                new int[0],
                true,
                true);
    }

    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name The configured name of the Limelight.
     * @param robotRotationSupplier Supplier for the current estimated robot rotation, used for MegaTag 2.
     * @param robotYawRateSupplierDegPerSec Supplier for the current robot yaw rate in degrees/sec.
     * @param robotToCameraSupplier Supplier for the current camera transform relative to the robot.
     * @param simpleTargetAllowedTagIds Allowed AprilTag IDs for simple target tracking. Empty allows all IDs.
     */
    public VisionIOLimelight(
            String name,
            Supplier<Rotation2d> robotRotationSupplier,
            DoubleSupplier robotYawRateSupplierDegPerSec,
            Supplier<Transform3d> robotToCameraSupplier,
            int[] simpleTargetAllowedTagIds) {
        this(
                name,
                robotRotationSupplier,
                robotYawRateSupplierDegPerSec,
                () -> true,
                robotToCameraSupplier,
                simpleTargetAllowedTagIds,
                true,
                true);
    }

    public VisionIOLimelight(
            String name,
            Supplier<Rotation2d> robotRotationSupplier,
            DoubleSupplier robotYawRateSupplierDegPerSec,
            BooleanSupplier robotOrientationTrustedSupplier,
            Supplier<Transform3d> robotToCameraSupplier,
            int[] simpleTargetAllowedTagIds) {
        this(
                name,
                robotRotationSupplier,
                robotYawRateSupplierDegPerSec,
                robotOrientationTrustedSupplier,
                robotToCameraSupplier,
                simpleTargetAllowedTagIds,
                true,
                true);
    }

    public VisionIOLimelight(
            String name,
            Supplier<Rotation2d> robotRotationSupplier,
            DoubleSupplier robotYawRateSupplierDegPerSec,
            Supplier<Transform3d> robotToCameraSupplier,
            int[] simpleTargetAllowedTagIds,
            boolean enableMegaTag1,
            boolean enableMegaTag2) {
        this(
                name,
                robotRotationSupplier,
                robotYawRateSupplierDegPerSec,
                () -> true,
                robotToCameraSupplier,
                simpleTargetAllowedTagIds,
                enableMegaTag1,
                enableMegaTag2);
    }

    /**
     * Creates a new VisionIOLimelight.
     *
     * @param name The configured name of the Limelight.
     * @param robotRotationSupplier Supplier for the current estimated robot rotation, used for MegaTag 2.
     * @param robotYawRateSupplierDegPerSec Supplier for the current robot yaw rate in degrees/sec.
     * @param robotOrientationTrustedSupplier Supplier for whether the orientation source is valid for MegaTag 2.
     * @param robotToCameraSupplier Supplier for the current camera transform relative to the robot.
     * @param simpleTargetAllowedTagIds Allowed AprilTag IDs for simple target tracking. Empty allows all IDs.
     * @param enableMegaTag1 Whether MegaTag 1 observations should be read.
     * @param enableMegaTag2 Whether MegaTag 2 observations should be read.
     */
    public VisionIOLimelight(
            String name,
            Supplier<Rotation2d> robotRotationSupplier,
            DoubleSupplier robotYawRateSupplierDegPerSec,
            BooleanSupplier robotOrientationTrustedSupplier,
            Supplier<Transform3d> robotToCameraSupplier,
            int[] simpleTargetAllowedTagIds,
            boolean enableMegaTag1,
            boolean enableMegaTag2) {
        this.name = Objects.requireNonNull(name, "name");
        this.robotRotationSupplier = Objects.requireNonNull(robotRotationSupplier, "robotRotationSupplier");
        this.robotYawRateSupplierDegPerSec =
                Objects.requireNonNull(robotYawRateSupplierDegPerSec, "robotYawRateSupplierDegPerSec");
        this.robotOrientationTrustedSupplier =
                Objects.requireNonNull(robotOrientationTrustedSupplier, "robotOrientationTrustedSupplier");
        this.robotToCameraSupplier = Objects.requireNonNull(robotToCameraSupplier, "robotToCameraSupplier");
        this.enableMegaTag1 = enableMegaTag1;
        this.enableMegaTag2 = enableMegaTag2;
        this.simpleTargetAllowedTagIds =
                Arrays.copyOf(
                        Objects.requireNonNull(simpleTargetAllowedTagIds, "simpleTargetAllowedTagIds"),
                        simpleTargetAllowedTagIds.length);
        for (int tagId : this.simpleTargetAllowedTagIds) {
            simpleTargetAllowedTagIdSet.add(tagId);
        }

        var table = NetworkTableInstance.getDefault().getTable(name);
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        captureLatencySubscriber = table.getDoubleTopic("cl").subscribe(0.0);
        tvSubscriber = table.getDoubleTopic("tv").subscribe(0.0);
        tidSubscriber = table.getDoubleTopic("tid").subscribe(-1.0);
        taSubscriber = table.getDoubleTopic("ta").subscribe(0.0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        boolean wasConnected = inputs.connected;

        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

        if (inputs.connected && (!wasConnected || !simpleTargetFilterConfigured)) {
            configureSimpleTargetFilter();
        } else if (!inputs.connected) {
            simpleTargetFilterConfigured = false;
        }

        // Update target observation
        boolean hasRawTarget = tvSubscriber.get() > 0.5;
        int latestTargetTagId = hasRawTarget ? (int) Math.round(tidSubscriber.get()) : -1;
        boolean targetAllowed = hasRawTarget && isSimpleTargetAllowed(latestTargetTagId);
        var txSample = txSubscriber.getAtomic();
        var tySample = tySubscriber.getAtomic();
        var pipelineLatencySample = latencySubscriber.getAtomic();
        var captureLatencySample = captureLatencySubscriber.getAtomic();
        double totalTargetLatencySeconds =
                (pipelineLatencySample.value + captureLatencySample.value) * 1.0e-3;
        long targetObservationTimestampMicros = Math.max(txSample.timestamp, tySample.timestamp);

        inputs.hasTarget = targetAllowed;
        inputs.latestTargetTagId = targetAllowed ? latestTargetTagId : -1;
        inputs.latestTargetArea = targetAllowed ? taSubscriber.get() : 0.0;
        inputs.latestTargetObservation =
                targetAllowed
                        ? new TargetObservation(
                                Rotation2d.fromDegrees(txSample.value),
                                Rotation2d.fromDegrees(tySample.value))
                        : new TargetObservation(new Rotation2d(), new Rotation2d());
        inputs.latestTargetObservationTimestampSeconds =
                targetAllowed && targetObservationTimestampMicros > 0L
                        ? targetObservationTimestampMicros * 1.0e-6 - totalTargetLatencySeconds
                        : Double.NaN;
        inputs.latestTargetObservationLatencySeconds =
                targetAllowed ? totalTargetLatencySeconds : 0.0;

        // Publish robot orientation and live camera pose for MegaTag 2 / moving cameras.
        Transform3d robotToCamera = robotToCameraSupplier.get();
        Rotation3d cameraRotation = robotToCamera.getRotation();
        LimelightHelpers.setCameraPose_RobotSpace(
                name,
                robotToCamera.getX(),
                robotToCamera.getY(),
                robotToCamera.getZ(),
                Units.radiansToDegrees(cameraRotation.getX()),
                Units.radiansToDegrees(cameraRotation.getY()),
                Units.radiansToDegrees(cameraRotation.getZ()));
        double robotYawRateDegPerSec = robotYawRateSupplierDegPerSec.getAsDouble();
        LimelightHelpers.SetRobotOrientation_NoFlush(
                name, robotRotationSupplier.get().getDegrees(), robotYawRateDegPerSec, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.Flush();
        boolean rejectMegaTag2ForUntrustedOrientation = !robotOrientationTrustedSupplier.getAsBoolean();

        // Read new pose observations from NetworkTables
        Set<Integer> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var rawSample : megatag1Subscriber.readQueue()) {
            if (!enableMegaTag1) continue;
            if (rawSample.value.length == 0) continue;
            for (int i = 11; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(new PoseObservation(
                    rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
                    parsePose(rawSample.value),
                    rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,
                    (int) rawSample.value[7],
                    rawSample.value[9],
                    PoseObservationType.MEGATAG_1));
        }
        boolean rejectMegaTag2ForYawRate = Math.abs(robotYawRateDegPerSec) > VisionConstants.maxMegaTag2YawRateDegPerSec;
        for (var rawSample : megatag2Subscriber.readQueue()) {
            if (!enableMegaTag2 || rejectMegaTag2ForYawRate || rejectMegaTag2ForUntrustedOrientation) continue;
            if (rawSample.value.length == 0) continue;
            for (int i = 11; i < rawSample.value.length; i += 7) {
                tagIds.add((int) rawSample.value[i]);
            }
            poseObservations.add(new PoseObservation(
                    rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,
                    parsePose(rawSample.value),
                    0.0,
                    (int) rawSample.value[7],
                    rawSample.value[9],
                    PoseObservationType.MEGATAG_2));
        }

        inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }

    private void configureSimpleTargetFilter() {
        if (simpleTargetAllowedTagIds.length > 0) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, simpleTargetAllowedTagIds);
        }
        simpleTargetFilterConfigured = true;
    }

    private boolean isSimpleTargetAllowed(int tagId) {
        return simpleTargetAllowedTagIdSet.isEmpty() || simpleTargetAllowedTagIdSet.contains(tagId);
    }

    /** Parses the 3D pose from a Limelight botpose array. */
    private static Pose3d parsePose(double[] rawLLArray) {
        return new Pose3d(
                rawLLArray[0],
                rawLLArray[1],
                rawLLArray[2],
                new Rotation3d(
                        Units.degreesToRadians(rawLLArray[3]),
                        Units.degreesToRadians(rawLLArray[4]),
                        Units.degreesToRadians(rawLLArray[5])));
    }

}
