package frc.robot.commands;

import static frc.robot.subsystems.shooter.turret.TurretConstants.kMaxAngleDeg;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMinAngleDeg;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.GenericConstants.AimPoints;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.LimelightHelpers;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheMoveCommand extends Command {
  private static final double TURRET_ANGLE_HISTORY_SECONDS = 1.0;
  private static final double MAX_TARGET_OBSERVATION_AGE_SECONDS = 0.25;
  private static final int VELOCITY_FILTER_SAMPLES =
      Math.max(
          1,
          (int)
              Math.round(
                  Constants.ShootOnTheMoveConstants.kVelocityFilterWindowSeconds
                      / Constants.kLoopPeriodSeconds));

  public enum TargetingMode {
    HUB,
    ALLIANCE_SIDE
  }

  private final DriveSubsystem drive;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final FlywheelSubsystem flywheel;
  private final TargetingMode targetingMode;
  private final VisionSubsystem vision;
  private final int turretVisionCameraIndex;
  private final Consumer<ShotSolution> shotSolutionConsumer;
  private final BooleanSupplier manualShotOverrideSupplier;
  private final DoubleSupplier manualHoodAngleSupplier;
  private final DoubleSupplier manualFlywheelRpmSupplier;
  private final Deque<TimedTurretAngleSample> turretAngleHistory = new ArrayDeque<>();

  private LinearFilter turretVelocityFilter = createVelocityFilter();
  private LinearFilter hoodVelocityFilter = createVelocityFilter();
  private LinearFilter turretVisionTxFilter = createTurretVisionTxFilter();

  private double lastTurretAngleDeg = 0.0;
  private double lastHoodAngleDeg = 0.0;
  private double filteredTurretVisionTxDeg = 0.0;
  private int lastTurretVisionTagId = -1;
  private ShotSolution latestSolution;

  private record TimedTurretAngleSample(double timestampSeconds, double angleDeg) {}

  public record ShotSolution(
      boolean valid,
      Translation2d target,
      Pose2d estimatedPose,
      Translation2d turretPosition,
      Translation2d lookaheadTurretPosition,
      double odometryDistanceMeters,
      double predictedDistanceMeters,
      boolean limelightDistanceAvailable,
      double limelightDistanceMeters,
      boolean turretVisionActive,
      int turretVisionTagId,
      double turretAngleDeg,
      Rotation2d turretAngle,
      double odometryTurretAngleDeg,
      Rotation2d leadCompensation,
      double turretVisionFilteredTxDeg,
      double turretVisionLineOfSightAngleDeg,
      double turretVelocityDegPerSecond,
      boolean manualShotOverrideActive,
      double hoodAngleDeg,
      double hoodVelocityDegPerSecond,
      double flywheelRpm) {}

  public ShootOnTheMoveCommand(
      DriveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      TargetingMode targetingMode,
      VisionSubsystem vision,
      int turretVisionCameraIndex,
      Consumer<ShotSolution> shotSolutionConsumer) {
    this(
        drive,
        turret,
        hood,
        targetingMode,
        vision,
        turretVisionCameraIndex,
        shotSolutionConsumer,
        () -> false,
        () -> 0.0,
        () -> 0.0);
  }

  public ShootOnTheMoveCommand(
      DriveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      TargetingMode targetingMode,
      VisionSubsystem vision,
      int turretVisionCameraIndex,
      Consumer<ShotSolution> shotSolutionConsumer,
      BooleanSupplier manualShotOverrideSupplier,
      DoubleSupplier manualHoodAngleSupplier,
      DoubleSupplier manualFlywheelRpmSupplier) {
    this(
        drive,
        turret,
        hood,
        null,
        targetingMode,
        vision,
        turretVisionCameraIndex,
        shotSolutionConsumer,
        manualShotOverrideSupplier,
        manualHoodAngleSupplier,
        manualFlywheelRpmSupplier);
  }

  public ShootOnTheMoveCommand(
      DriveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      FlywheelSubsystem flywheel,
      TargetingMode targetingMode,
      VisionSubsystem vision,
      int turretVisionCameraIndex,
      Consumer<ShotSolution> shotSolutionConsumer) {
    this(
        drive,
        turret,
        hood,
        flywheel,
        targetingMode,
        vision,
        turretVisionCameraIndex,
        shotSolutionConsumer,
        () -> false,
        () -> 0.0,
        () -> 0.0);
  }

  public ShootOnTheMoveCommand(
      DriveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      FlywheelSubsystem flywheel,
      TargetingMode targetingMode,
      VisionSubsystem vision,
      int turretVisionCameraIndex,
      Consumer<ShotSolution> shotSolutionConsumer,
      BooleanSupplier manualShotOverrideSupplier,
      DoubleSupplier manualHoodAngleSupplier,
      DoubleSupplier manualFlywheelRpmSupplier) {
    this.drive = Objects.requireNonNull(drive, "drive");
    this.turret = Objects.requireNonNull(turret, "turret");
    this.hood = Objects.requireNonNull(hood, "hood");
    this.flywheel = flywheel;
    this.targetingMode = Objects.requireNonNull(targetingMode, "targetingMode");
    this.vision = vision;
    this.turretVisionCameraIndex = turretVisionCameraIndex;
    this.shotSolutionConsumer =
        shotSolutionConsumer != null ? shotSolutionConsumer : (solution) -> {};
    this.manualShotOverrideSupplier =
        manualShotOverrideSupplier != null ? manualShotOverrideSupplier : () -> false;
    this.manualHoodAngleSupplier =
        manualHoodAngleSupplier != null ? manualHoodAngleSupplier : () -> 0.0;
    this.manualFlywheelRpmSupplier =
        manualFlywheelRpmSupplier != null ? manualFlywheelRpmSupplier : () -> 0.0;

    setName("ShootOnTheMove/" + targetingMode.name());
    if (flywheel != null) {
      addRequirements(turret, hood, flywheel);
    } else {
      addRequirements(turret, hood);
    }
  }

  @Override
  public void initialize() {
    turretVelocityFilter = createVelocityFilter();
    hoodVelocityFilter = createVelocityFilter();
    turretVisionTxFilter = createTurretVisionTxFilter();
    lastTurretAngleDeg = turret.getAngle();
    lastHoodAngleDeg = hood.getAngle();
    filteredTurretVisionTxDeg = 0.0;
    lastTurretVisionTagId = -1;
    latestSolution = null;
    turretAngleHistory.clear();
    recordTurretAngleSample(Timer.getFPGATimestamp(), lastTurretAngleDeg);
    shotSolutionConsumer.accept(null);
  }

  @Override
  public void execute() {
    recordTurretAngleSample(Timer.getFPGATimestamp(), turret.getAngle());

    Translation2d target =
        switch (targetingMode) {
          case HUB -> AimPoints.getAllianceHubPosition().toTranslation2d();
          case ALLIANCE_SIDE -> AimPoints.getAllianceFarSidePosition().toTranslation2d();
        };

    if (target == null) {
      latestSolution = null;
      shotSolutionConsumer.accept(null);
      Logger.recordOutput("ShootOnTheMove/HasTarget", false);
      Logger.recordOutput("ShootOnTheMove/ValidShot", false);
      clearDistanceLogs();
      return;
    }

    latestSolution = calculateShotSolution(target);

    turret.setTurretAngle(latestSolution.turretAngleDeg());
    hood.setHoodAngle(latestSolution.hoodAngleDeg());
    shotSolutionConsumer.accept(latestSolution);
    if (flywheel != null) {
      flywheel.setVelocity(latestSolution.flywheelRpm());
    }

    Logger.recordOutput("ShootOnTheMove/HasTarget", true);
    Logger.recordOutput("ShootOnTheMove/IsHubTargeting", targetingMode == TargetingMode.HUB);
    Logger.recordOutput("ShootOnTheMove/Target", new Pose2d(latestSolution.target(), Rotation2d.kZero));
    Logger.recordOutput("ShootOnTheMove/EstimatedRobotPose", latestSolution.estimatedPose());
    Logger.recordOutput(
        "ShootOnTheMove/TurretPosition",
        new Pose2d(latestSolution.turretPosition(), Rotation2d.kZero));
    Logger.recordOutput(
        "ShootOnTheMove/PredictedTurretPosition",
        new Pose2d(latestSolution.lookaheadTurretPosition(), Rotation2d.kZero));
    Logger.recordOutput("ShootOnTheMove/DistanceMeters", latestSolution.predictedDistanceMeters());
    Logger.recordOutput(
        "ShootOnTheMove/OdometryHubToTurretPivotDistanceMeters",
        latestSolution.odometryDistanceMeters());
    Logger.recordOutput(
        "ShootOnTheMove/PredictedHubToTurretPivotDistanceMeters",
        latestSolution.predictedDistanceMeters());
    Logger.recordOutput(
        "ShootOnTheMove/LimelightHubToTurretPivotDistanceAvailable",
        latestSolution.limelightDistanceAvailable());
    Logger.recordOutput(
        "ShootOnTheMove/LimelightHubToTurretPivotDistanceMeters",
        latestSolution.limelightDistanceMeters());
    Logger.recordOutput(
        "ShootOnTheMove/LimelightMinusOdometryHubDistanceMeters",
        latestSolution.limelightDistanceAvailable()
            ? latestSolution.limelightDistanceMeters() - latestSolution.odometryDistanceMeters()
            : Double.NaN);
    Logger.recordOutput("ShootOnTheMove/TurretVisionActive", latestSolution.turretVisionActive());
    Logger.recordOutput("ShootOnTheMove/TurretVisionTagId", latestSolution.turretVisionTagId());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionTargetArea",
        latestSolution.turretVisionActive() && vision != null && turretVisionCameraIndex >= 0
            ? vision.getTargetArea(turretVisionCameraIndex)
            : 0.0);
    Logger.recordOutput("ShootOnTheMove/TurretAngleDeg", latestSolution.turretAngleDeg());
    Logger.recordOutput(
        "ShootOnTheMove/OdometryTurretAngleDeg", latestSolution.odometryTurretAngleDeg());
    Logger.recordOutput(
        "ShootOnTheMove/TurretLeadCompensationDeg",
        latestSolution.leadCompensation().getDegrees());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionFilteredTxDeg", latestSolution.turretVisionFilteredTxDeg());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionLineOfSightAngleDeg",
        latestSolution.turretVisionLineOfSightAngleDeg());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVelocityDegPerSec", latestSolution.turretVelocityDegPerSecond());
    Logger.recordOutput(
        "ShootOnTheMove/ManualShotOverrideActive", latestSolution.manualShotOverrideActive());
    Logger.recordOutput("ShootOnTheMove/HoodAngleDeg", latestSolution.hoodAngleDeg());
    Logger.recordOutput(
        "ShootOnTheMove/HoodVelocityDegPerSec", latestSolution.hoodVelocityDegPerSecond());
    Logger.recordOutput("ShootOnTheMove/FlywheelSetpointRPM", latestSolution.flywheelRpm());
    Logger.recordOutput("ShootOnTheMove/ValidShot", latestSolution.valid());
  }

  @Override
  public void end(boolean interrupted) {
    latestSolution = null;
    shotSolutionConsumer.accept(null);
    turretAngleHistory.clear();
    turret.stop();
    hood.stop();
    if (flywheel != null) {
      flywheel.stop();
    }
    Logger.recordOutput("ShootOnTheMove/HasTarget", false);
    Logger.recordOutput("ShootOnTheMove/ValidShot", false);
    clearDistanceLogs();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private ShotSolution calculateShotSolution(Translation2d target) {
    Pose2d estimatedPose = getLatencyCompensatedPose();
    Rotation2d turretZeroFieldRotation =
        estimatedPose.getRotation().plus(Constants.robotToTurret.getRotation().toRotation2d());
    Translation2d turretOffsetField =
        Constants.ShootOnTheMoveConstants.kRobotToTurret.rotateBy(estimatedPose.getRotation());
    Translation2d turretPosition = estimatedPose.getTranslation().plus(turretOffsetField);
    Translation2d turretFieldVelocity =
        calculateTurretFieldVelocity(drive.getFieldRelativeSpeeds(), turretOffsetField);
    double odometryDistance = turretPosition.getDistance(target);

    Translation2d lookaheadTurretPosition = turretPosition;
    double lookaheadDistance = odometryDistance;
    for (int i = 0; i < Constants.ShootOnTheMoveConstants.kLookaheadIterations; i++) {
      double timeOfFlight =
          sampleInterpolatedValue(
              Constants.ShootOnTheMoveConstants.kTimeOfFlightSecondsTable, lookaheadDistance);
      lookaheadTurretPosition =
          turretPosition.plus(
              new Translation2d(
                  turretFieldVelocity.getX() * timeOfFlight,
                  turretFieldVelocity.getY() * timeOfFlight));
      lookaheadDistance = lookaheadTurretPosition.getDistance(target);
    }
    double shotTimeOfFlightSeconds =
        sampleInterpolatedValue(
            Constants.ShootOnTheMoveConstants.kTimeOfFlightSecondsTable, lookaheadDistance);

    double currentTurretAngleDeg = turret.getAngle();
    Rotation2d odometryLookaheadTurretAngle =
        target.minus(lookaheadTurretPosition).getAngle().minus(turretZeroFieldRotation);
    double odometryLookaheadTurretAngleDeg =
        applyCableWrapLimits(odometryLookaheadTurretAngle.getDegrees());
    Rotation2d odometryLineOfSightTurretAngle =
        target.minus(turretPosition).getAngle().minus(turretZeroFieldRotation);
    double odometryLineOfSightTurretAngleDeg =
        applyCableWrapLimits(odometryLineOfSightTurretAngle.getDegrees());
    Rotation2d leadCompensation =
        odometryLookaheadTurretAngle.minus(odometryLineOfSightTurretAngle);

    Rotation2d turretAngle = odometryLookaheadTurretAngle;
    double turretAngleDeg = odometryLookaheadTurretAngleDeg;
    boolean turretVisionActive = shouldUseTurretVision();
    double turretVisionObservationTimestampSeconds =
        turretVisionActive
            ? vision.getTargetObservationTimestampSeconds(turretVisionCameraIndex)
            : Double.NaN;
    double turretVisionObservationAgeSeconds =
        Double.isFinite(turretVisionObservationTimestampSeconds)
            ? Timer.getFPGATimestamp() - turretVisionObservationTimestampSeconds
            : Double.NaN;
    if (turretVisionActive
        && (!Double.isFinite(turretVisionObservationTimestampSeconds)
            || turretVisionObservationAgeSeconds < 0.0
            || turretVisionObservationAgeSeconds > MAX_TARGET_OBSERVATION_AGE_SECONDS)) {
      turretVisionActive = false;
    }
    int turretVisionTagId = turretVisionActive ? vision.getTargetTagId(turretVisionCameraIndex) : -1;
    double limelightDistanceMeters = Double.NaN;
    boolean limelightDistanceAvailable = false;
    double turretVisionLineOfSightAngleDeg = 0.0;
    double turretVisionObservedTurretAngleDeg = currentTurretAngleDeg;

    if (turretVisionActive) {
      limelightDistanceMeters =
          calculateLimelightHubDistanceMeters(estimatedPose, target, turretVisionTagId);
      limelightDistanceAvailable = Double.isFinite(limelightDistanceMeters);
      double rawTurretVisionTxDeg = vision.getTargetX(turretVisionCameraIndex).getDegrees();
      if (lastTurretVisionTagId != turretVisionTagId) {
        turretVisionTxFilter = createTurretVisionTxFilter();
        filteredTurretVisionTxDeg = rawTurretVisionTxDeg;
      } else {
        filteredTurretVisionTxDeg = turretVisionTxFilter.calculate(rawTurretVisionTxDeg);
      }
      lastTurretVisionTagId = turretVisionTagId;

      double turretVisionTxDeg = applyTurretVisionTxDeadband(filteredTurretVisionTxDeg);
      turretVisionObservedTurretAngleDeg =
          getTurretAngleAtTimestamp(turretVisionObservationTimestampSeconds, currentTurretAngleDeg);
      Rotation2d turretVisionLineOfSightAngle =
          Rotation2d.fromDegrees(turretVisionObservedTurretAngleDeg)
              .minus(Rotation2d.fromDegrees(turretVisionTxDeg))
              .plus(
                  Rotation2d.fromDegrees(
                      Constants.ShootOnTheMoveConstants.kTurretVisionAimOffsetDegrees));
      turretVisionLineOfSightAngleDeg =
          applyCableWrapLimits(turretVisionLineOfSightAngle.getDegrees());
      turretAngle = turretVisionLineOfSightAngle.plus(leadCompensation);
      turretAngleDeg = applyCableWrapLimits(turretAngle.getDegrees());
    } else {
      filteredTurretVisionTxDeg = 0.0;
      lastTurretVisionTagId = -1;
    }

    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionObservationTimestampSeconds",
        turretVisionObservationTimestampSeconds);
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionObservationAgeSeconds", turretVisionObservationAgeSeconds);
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionObservationLatencySeconds",
        turretVisionActive && vision != null && turretVisionCameraIndex >= 0
            ? vision.getTargetObservationLatencySeconds(turretVisionCameraIndex)
            : 0.0);
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionObservedTurretAngleDeg", turretVisionObservedTurretAngleDeg);

    boolean manualShotOverrideActive = manualShotOverrideSupplier.getAsBoolean();
    double hoodAngleDeg =
        sampleInterpolatedValue(
            Constants.ShootOnTheMoveConstants.kHoodAngleDegreesTable, lookaheadDistance);
    double flywheelRpm =
        sampleInterpolatedValue(
            Constants.ShootOnTheMoveConstants.kFlywheelSpeedRpmTable, lookaheadDistance);
    if (manualShotOverrideActive) {
      hoodAngleDeg = manualHoodAngleSupplier.getAsDouble();
      flywheelRpm = manualFlywheelRpmSupplier.getAsDouble();
    }
    boolean shotIsValid =
        Double.isFinite(lookaheadDistance)
            && Double.isFinite(turretAngleDeg)
            && Double.isFinite(shotTimeOfFlightSeconds)
            && shotTimeOfFlightSeconds > 0.0
            && Double.isFinite(hoodAngleDeg)
            && Double.isFinite(flywheelRpm);
    double turretVelocityDegPerSecond =
        turretVelocityFilter.calculate(
            shortestAngleDeltaDegrees(turretAngleDeg, lastTurretAngleDeg)
                / Constants.kLoopPeriodSeconds);
    double hoodVelocityDegPerSecond =
        hoodVelocityFilter.calculate((hoodAngleDeg - lastHoodAngleDeg) / Constants.kLoopPeriodSeconds);

    lastTurretAngleDeg = turretAngleDeg;
    lastHoodAngleDeg = hoodAngleDeg;

    return new ShotSolution(
        shotIsValid,
        target,
        estimatedPose,
        turretPosition,
        lookaheadTurretPosition,
        odometryDistance,
        lookaheadDistance,
        limelightDistanceAvailable,
        limelightDistanceMeters,
        turretVisionActive,
        turretVisionTagId,
        turretAngleDeg,
        turretAngle,
        odometryLineOfSightTurretAngleDeg,
        leadCompensation,
        filteredTurretVisionTxDeg,
        turretVisionLineOfSightAngleDeg,
        turretVelocityDegPerSecond,
        manualShotOverrideActive,
        hoodAngleDeg,
        hoodVelocityDegPerSecond,
        flywheelRpm);
  }

  private Pose2d getLatencyCompensatedPose() {
    Pose2d estimatedPose = drive.getPose();
    ChassisSpeeds robotRelativeSpeeds = drive.getRobotRelativeSpeeds();
    return estimatedPose.exp(
        new Twist2d(
            robotRelativeSpeeds.vxMetersPerSecond
                * Constants.ShootOnTheMoveConstants.kPhaseDelaySeconds,
            robotRelativeSpeeds.vyMetersPerSecond
                * Constants.ShootOnTheMoveConstants.kPhaseDelaySeconds,
            robotRelativeSpeeds.omegaRadiansPerSecond
                * Constants.ShootOnTheMoveConstants.kPhaseDelaySeconds));
  }

  private Translation2d calculateTurretFieldVelocity(
      ChassisSpeeds fieldRelativeSpeeds, Translation2d turretOffsetField) {
    Translation2d robotLinearVelocity =
        new Translation2d(
            fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    Translation2d rotationalVelocity =
        new Translation2d(
            -fieldRelativeSpeeds.omegaRadiansPerSecond * turretOffsetField.getY(),
            fieldRelativeSpeeds.omegaRadiansPerSecond * turretOffsetField.getX());
    return robotLinearVelocity.plus(rotationalVelocity);
  }

  private static double sampleInterpolatedValue(double[][] points, double distanceMeters) {
    if (points.length == 0) {
      return Double.NaN;
    }
    if (points.length == 1) {
      return points[0][1];
    }

    if (distanceMeters <= points[0][0]) {
      return extrapolateLinearly(points[0], points[1], distanceMeters);
    }
    int lastIndex = points.length - 1;
    if (distanceMeters >= points[lastIndex][0]) {
      return extrapolateLinearly(points[lastIndex - 1], points[lastIndex], distanceMeters);
    }

    for (int i = 1; i < points.length; i++) {
      if (distanceMeters <= points[i][0]) {
        return extrapolateLinearly(points[i - 1], points[i], distanceMeters);
      }
    }

    return Double.NaN;
  }

  private static double extrapolateLinearly(
      double[] lowerPoint, double[] upperPoint, double distanceMeters) {
    if (lowerPoint == null || upperPoint == null || lowerPoint.length < 2 || upperPoint.length < 2) {
      return Double.NaN;
    }

    double lowerDistance = lowerPoint[0];
    double upperDistance = upperPoint[0];
    if (Math.abs(upperDistance - lowerDistance) <= 1.0e-9) {
      return upperPoint[1];
    }

    double interpolationFraction = (distanceMeters - lowerDistance) / (upperDistance - lowerDistance);
    return lowerPoint[1] + (upperPoint[1] - lowerPoint[1]) * interpolationFraction;
  }

  private static LinearFilter createVelocityFilter() {
    return LinearFilter.movingAverage(VELOCITY_FILTER_SAMPLES);
  }

  private static LinearFilter createTurretVisionTxFilter() {
    return LinearFilter.singlePoleIIR(
        Constants.ShootOnTheMoveConstants.kTurretVisionTxFilterTimeConstantSeconds,
        Constants.kLoopPeriodSeconds);
  }

  private static double shortestAngleDeltaDegrees(double currentAngleDeg, double previousAngleDeg) {
    return MathUtil.inputModulus(currentAngleDeg - previousAngleDeg, -180, 180.0);
  }

  private static double applyCableWrapLimits(double angleDeg) {
    // Rotation2d math gives us a circular angle. Shift it once across the cable-wrap seam so the
    // commanded turret angle always stays in the physical [-225, 135] window.
    if (angleDeg > kMaxAngleDeg) {
      return angleDeg - 360.0;
    }
    if (angleDeg < kMinAngleDeg) {
      return angleDeg + 360.0;
    }
    return angleDeg;
  }

  private static double applyTurretVisionTxDeadband(double txDeg) {
    return Math.abs(txDeg) < Constants.ShootOnTheMoveConstants.kTurretVisionTxDeadbandDegrees
        ? 0.0
        : txDeg;
  }

  private void recordTurretAngleSample(double timestampSeconds, double angleDeg) {
    turretAngleHistory.addLast(new TimedTurretAngleSample(timestampSeconds, angleDeg));
    while (turretAngleHistory.size() > 1
        && turretAngleHistory.peekFirst().timestampSeconds()
            < timestampSeconds - TURRET_ANGLE_HISTORY_SECONDS) {
      turretAngleHistory.removeFirst();
    }
  }

  private double getTurretAngleAtTimestamp(
      double timestampSeconds, double fallbackAngleDeg) {
    if (!Double.isFinite(timestampSeconds) || turretAngleHistory.isEmpty()) {
      return fallbackAngleDeg;
    }

    TimedTurretAngleSample firstSample = turretAngleHistory.peekFirst();
    TimedTurretAngleSample lastSample = turretAngleHistory.peekLast();
    if (timestampSeconds <= firstSample.timestampSeconds()) {
      return firstSample.angleDeg();
    }
    if (timestampSeconds >= lastSample.timestampSeconds()) {
      return lastSample.angleDeg();
    }

    TimedTurretAngleSample previousSample = firstSample;
    for (TimedTurretAngleSample sample : turretAngleHistory) {
      if (sample.timestampSeconds() >= timestampSeconds) {
        double sampleWindowSeconds =
            sample.timestampSeconds() - previousSample.timestampSeconds();
        if (sampleWindowSeconds <= 1.0e-9) {
          return sample.angleDeg();
        }

        double interpolationFraction =
            (timestampSeconds - previousSample.timestampSeconds()) / sampleWindowSeconds;
        return previousSample.angleDeg()
            + (sample.angleDeg() - previousSample.angleDeg()) * interpolationFraction;
      }
      previousSample = sample;
    }

    return fallbackAngleDeg;
  }

  private double calculateLimelightHubDistanceMeters(
      Pose2d estimatedPose, Translation2d hubTarget, int targetTagId) {
    if (vision == null || turretVisionCameraIndex < 0 || targetTagId < 0) {
      return Double.NaN;
    }

    String cameraName = VisionConstants.getCameraName(turretVisionCameraIndex);
    var tagPose = VisionConstants.aprilTagLayout.getTagPose(targetTagId);
    if (cameraName.isEmpty() || tagPose.isEmpty()) {
      return Double.NaN;
    }

    Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(cameraName);
    Translation2d tagTranslationRobot = targetPoseRobotSpace.getTranslation().toTranslation2d();
    if (tagTranslationRobot.getNorm() < 1.0e-6) {
      return Double.NaN;
    }

    Translation2d hubOffsetField =
        hubTarget.minus(tagPose.get().getTranslation().toTranslation2d());
    Translation2d hubOffsetRobot = hubOffsetField.rotateBy(estimatedPose.getRotation().unaryMinus());
    Translation2d hubTranslationRobot = tagTranslationRobot.plus(hubOffsetRobot);
    Translation2d turretPivotRobot =
        new Translation2d(Constants.robotToTurret.getX(), Constants.robotToTurret.getY());
    return hubTranslationRobot.getDistance(turretPivotRobot);
  }

  private boolean shouldUseTurretVision() {
    if (targetingMode != TargetingMode.HUB || vision == null || turretVisionCameraIndex < 0) {
      return false;
    }

    if (!vision.hasTarget(turretVisionCameraIndex)) {
      return false;
    }

    return VisionConstants.isAllianceHubTargetTag(vision.getTargetTagId(turretVisionCameraIndex));
  }

  private void clearDistanceLogs() {
    Logger.recordOutput("ShootOnTheMove/DistanceMeters", Double.NaN);
    Logger.recordOutput("ShootOnTheMove/OdometryHubToTurretPivotDistanceMeters", Double.NaN);
    Logger.recordOutput("ShootOnTheMove/PredictedHubToTurretPivotDistanceMeters", Double.NaN);
    Logger.recordOutput("ShootOnTheMove/LimelightHubToTurretPivotDistanceAvailable", false);
    Logger.recordOutput("ShootOnTheMove/LimelightHubToTurretPivotDistanceMeters", Double.NaN);
    Logger.recordOutput("ShootOnTheMove/LimelightMinusOdometryHubDistanceMeters", Double.NaN);
  }
}
