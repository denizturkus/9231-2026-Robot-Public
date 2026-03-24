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
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheMoveCommand extends Command {
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

  private LinearFilter turretVelocityFilter = createVelocityFilter();
  private LinearFilter hoodVelocityFilter = createVelocityFilter();
  private LinearFilter turretVisionTxFilter = createTurretVisionTxFilter();

  private double lastTurretAngleDeg = 0.0;
  private double lastHoodAngleDeg = 0.0;
  private double filteredTurretVisionTxDeg = 0.0;
  private int lastTurretVisionTagId = -1;
  private ShotSolution latestSolution;

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
    shotSolutionConsumer.accept(null);
  }

  @Override
  public void execute() {
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
          Constants.ShootOnTheMoveConstants.kTimeOfFlightSeconds.get(
              clampDistance(lookaheadDistance));
      lookaheadTurretPosition =
          turretPosition.plus(
              new Translation2d(
                  turretFieldVelocity.getX() * timeOfFlight,
                  turretFieldVelocity.getY() * timeOfFlight));
      lookaheadDistance = lookaheadTurretPosition.getDistance(target);
    }

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
    boolean turretVisionActive = false; //shouldUseTurretVision(); TODO: fix turret vision sometime later
    int turretVisionTagId = turretVisionActive ? vision.getTargetTagId(turretVisionCameraIndex) : -1;
    double limelightDistanceMeters = Double.NaN;
    boolean limelightDistanceAvailable = false;
    double turretVisionLineOfSightAngleDeg = 0.0;

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
      Rotation2d turretVisionLineOfSightAngle =
          Rotation2d.fromDegrees(currentTurretAngleDeg)
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

    boolean manualShotOverrideActive = manualShotOverrideSupplier.getAsBoolean();
    double hoodAngleDeg =
        Constants.ShootOnTheMoveConstants.kHoodAngleDegrees.get(clampDistance(lookaheadDistance));
    double flywheelRpm =
        Constants.ShootOnTheMoveConstants.kFlywheelSpeedRpm.get(clampDistance(lookaheadDistance));
    if (manualShotOverrideActive) {
      hoodAngleDeg = manualHoodAngleSupplier.getAsDouble();
      flywheelRpm = manualFlywheelRpmSupplier.getAsDouble();
    }
    double turretVelocityDegPerSecond =
        turretVelocityFilter.calculate(
            shortestAngleDeltaDegrees(turretAngleDeg, lastTurretAngleDeg)
                / Constants.kLoopPeriodSeconds);
    double hoodVelocityDegPerSecond =
        hoodVelocityFilter.calculate((hoodAngleDeg - lastHoodAngleDeg) / Constants.kLoopPeriodSeconds);

    lastTurretAngleDeg = turretAngleDeg;
    lastHoodAngleDeg = hoodAngleDeg;

    return new ShotSolution(
        lookaheadDistance >= Constants.ShootOnTheMoveConstants.kMinDistanceMeters
            && lookaheadDistance <= Constants.ShootOnTheMoveConstants.kMaxDistanceMeters,
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

  private static double clampDistance(double distanceMeters) {
    return MathUtil.clamp(
        distanceMeters,
        Constants.ShootOnTheMoveConstants.kMinDistanceMeters,
        Constants.ShootOnTheMoveConstants.kMaxDistanceMeters);
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

  private double calculateLimelightHubDistanceMeters(
      Pose2d estimatedPose, Translation2d hubTarget, int targetTagId) {
    if (vision == null || turretVisionCameraIndex < 0 || targetTagId < 0) {
      return Double.NaN;
    }

    String cameraName = "limelight-turret";
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
