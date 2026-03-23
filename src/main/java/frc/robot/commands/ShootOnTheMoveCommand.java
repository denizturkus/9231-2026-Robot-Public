package frc.robot.commands;

import static frc.robot.subsystems.shooter.turret.TurretConstants.kMaxAngleDeg;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kMinAngleDeg;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheMoveCommand extends Command {
  private static final double TURRET_ANGLE_LIMIT_EPSILON_DEG = 1e-9;
  private static final int VELOCITY_FILTER_SAMPLES =
      Math.max(
          1,
          (int)
              Math.round(
                  Constants.ShootOnTheMoveConstants.kVelocityFilterWindowSeconds
                      / Constants.kLoopPeriodSeconds));
  private static final int TURRET_VISION_ACQUIRE_LOOPS =
      loopsForSeconds(Constants.ShootOnTheMoveConstants.kTurretVisionAcquireSeconds);

  private final DriveSubsystem drive;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final FlywheelSubsystem flywheel;
  private final Supplier<Translation2d> targetSupplier;
  private final VisionSubsystem vision;
  private final int turretVisionCameraIndex;
  private final Consumer<ShotSolution> shotSolutionConsumer;

  private LinearFilter turretVelocityFilter = createVelocityFilter();
  private LinearFilter hoodVelocityFilter = createVelocityFilter();
  private LinearFilter turretVisionTxFilter = createTurretVisionTxFilter();

  private double lastTurretAngleDeg = 0.0;
  private double lastHoodAngleDeg = 0.0;
  private boolean turretVisionLocked = false;
  private int turretVisionSeenLoops = 0;
  private int turretVisionMissedLoops = 0;
  private double filteredTurretVisionTxDeg = 0.0;
  private int lastTurretVisionTagId = -1;
  private ShotSolution latestSolution;

  public record ShotSolution(
      boolean valid,
      Translation2d target,
      Pose2d estimatedPose,
      Translation2d turretPosition,
      Translation2d lookaheadTurretPosition,
      double distanceMeters,
      boolean turretVisionRawAvailable,
      boolean turretVisionActive,
      int turretVisionTagId,
      double turretAngleDeg,
      Rotation2d turretAngle,
      double odometryTurretAngleDeg,
      Rotation2d odometryTurretAngle,
      Rotation2d leadCompensation,
      Rotation2d turretVisionTx,
      double turretVisionFilteredTxDeg,
      double turretVisionLineOfSightAngleDeg,
      Rotation2d turretVisionLineOfSightAngle,
      double turretVisionAppliedCorrectionDeg,
      boolean turretVisionCorrectionRejected,
      double turretVelocityDegPerSecond,
      double hoodAngleDeg,
      double hoodVelocityDegPerSecond,
      double flywheelRpm) {}

  public ShootOnTheMoveCommand(
      DriveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      FlywheelSubsystem flywheel) {
    this(drive, turret, hood, flywheel, hubTargetSupplier(), null, -1, null);
  }

  public ShootOnTheMoveCommand(
      DriveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      FlywheelSubsystem flywheel,
      Supplier<Translation2d> targetSupplier) {
    this(drive, turret, hood, flywheel, targetSupplier, null, -1, null);
  }

  public ShootOnTheMoveCommand(
      DriveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      FlywheelSubsystem flywheel,
      Supplier<Translation2d> targetSupplier,
      VisionSubsystem vision,
      int turretVisionCameraIndex) {
    this(drive, turret, hood, flywheel, targetSupplier, vision, turretVisionCameraIndex, null);
  }

  public ShootOnTheMoveCommand(
      DriveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      Supplier<Translation2d> targetSupplier,
      VisionSubsystem vision,
      int turretVisionCameraIndex,
      Consumer<ShotSolution> shotSolutionConsumer) {
    this(drive, turret, hood, null, targetSupplier, vision, turretVisionCameraIndex, shotSolutionConsumer);
  }

  private ShootOnTheMoveCommand(
      DriveSubsystem drive,
      TurretSubsystem turret,
      HoodSubsystem hood,
      FlywheelSubsystem flywheel,
      Supplier<Translation2d> targetSupplier,
      VisionSubsystem vision,
      int turretVisionCameraIndex,
      Consumer<ShotSolution> shotSolutionConsumer) {
    this.drive = Objects.requireNonNull(drive, "drive");
    this.turret = Objects.requireNonNull(turret, "turret");
    this.hood = Objects.requireNonNull(hood, "hood");
    this.flywheel = flywheel;
    this.targetSupplier = Objects.requireNonNull(targetSupplier, "targetSupplier");
    this.vision = vision;
    this.turretVisionCameraIndex = turretVisionCameraIndex;
    this.shotSolutionConsumer =
        shotSolutionConsumer != null ? shotSolutionConsumer : (solution) -> {};

    setName("ShootOnTheMove");
    if (flywheel != null) {
      addRequirements(turret, hood, flywheel);
    } else {
      addRequirements(turret, hood);
    }
  }

  public static Supplier<Translation2d> hubTargetSupplier() {
    return () -> AimPoints.getAllianceHubPosition().toTranslation2d();
  }

  public static Supplier<Translation2d> allianceSideTargetSupplier() {
    return () -> AimPoints.getAllianceFarSidePosition().toTranslation2d();
  }

  public static Supplier<Translation2d> targetSupplierFor(Translation2d target) {
    Translation2d fixedTarget = Objects.requireNonNull(target, "target");
    return () -> fixedTarget;
  }

  @Override
  public void initialize() {
    turretVelocityFilter = createVelocityFilter();
    hoodVelocityFilter = createVelocityFilter();
    turretVisionTxFilter = createTurretVisionTxFilter();
    lastTurretAngleDeg = turret.getAngle();
    lastHoodAngleDeg = hood.getAngle();
    turretVisionLocked = false;
    turretVisionSeenLoops = 0;
    turretVisionMissedLoops = 0;
    filteredTurretVisionTxDeg = 0.0;
    lastTurretVisionTagId = -1;
    latestSolution = null;
    shotSolutionConsumer.accept(null);
  }

  @Override
  public void execute() {
    Translation2d target = targetSupplier.get();
    if (target == null) {
      latestSolution = null;
      shotSolutionConsumer.accept(null);
      Logger.recordOutput("ShootOnTheMove/HasTarget", false);
      Logger.recordOutput("ShootOnTheMove/ValidShot", false);
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
    Logger.recordOutput("ShootOnTheMove/Target", new Pose2d(latestSolution.target(), Rotation2d.kZero));
    Logger.recordOutput("ShootOnTheMove/EstimatedRobotPose", latestSolution.estimatedPose());
    Logger.recordOutput(
        "ShootOnTheMove/TurretPosition",
        new Pose2d(latestSolution.turretPosition(), Rotation2d.kZero));
    Logger.recordOutput(
        "ShootOnTheMove/PredictedTurretPosition",
        new Pose2d(latestSolution.lookaheadTurretPosition(), Rotation2d.kZero));
    Logger.recordOutput("ShootOnTheMove/DistanceMeters", latestSolution.distanceMeters());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionRawAvailable", latestSolution.turretVisionRawAvailable());
    Logger.recordOutput("ShootOnTheMove/TurretVisionActive", latestSolution.turretVisionActive());
    Logger.recordOutput("ShootOnTheMove/TurretVisionTagId", latestSolution.turretVisionTagId());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionTargetArea", vision != null && turretVisionCameraIndex >= 0
            ? vision.getTargetArea(turretVisionCameraIndex)
            : 0.0);
    Logger.recordOutput("ShootOnTheMove/TurretAngleDeg", latestSolution.turretAngleDeg());
    Logger.recordOutput(
        "ShootOnTheMove/OdometryTurretAngleDeg", latestSolution.odometryTurretAngleDeg());
    Logger.recordOutput(
        "ShootOnTheMove/TurretLeadCompensationDeg",
        latestSolution.leadCompensation().getDegrees());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionTxDeg", latestSolution.turretVisionTx().getDegrees());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionFilteredTxDeg", latestSolution.turretVisionFilteredTxDeg());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionLineOfSightAngleDeg",
        latestSolution.turretVisionLineOfSightAngleDeg());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionAppliedCorrectionDeg",
        latestSolution.turretVisionAppliedCorrectionDeg());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionCorrectionRejected",
        latestSolution.turretVisionCorrectionRejected());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVelocityDegPerSec", latestSolution.turretVelocityDegPerSecond());
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
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private ShotSolution calculateShotSolution(Translation2d target) {
    Pose2d estimatedPose = getLatencyCompensatedPose();
    Translation2d turretOffsetField =
        Constants.ShootOnTheMoveConstants.kRobotToTurret.rotateBy(estimatedPose.getRotation());
    Translation2d turretPosition = estimatedPose.getTranslation().plus(turretOffsetField);
    Translation2d turretFieldVelocity =
        calculateTurretFieldVelocity(drive.getFieldRelativeSpeeds(), turretOffsetField);

    Translation2d lookaheadTurretPosition = turretPosition;
    double lookaheadDistance = turretPosition.getDistance(target);
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
        target.minus(lookaheadTurretPosition).getAngle().minus(estimatedPose.getRotation());
    double odometryLookaheadTurretAngleDeg =
        resolveTurretAngleToLimits(
            odometryLookaheadTurretAngle.getDegrees(), currentTurretAngleDeg);
    Rotation2d odometryLineOfSightTurretAngle =
        target.minus(turretPosition).getAngle().minus(estimatedPose.getRotation());
    double odometryLineOfSightTurretAngleDeg =
        resolveTurretAngleToLimits(
            odometryLineOfSightTurretAngle.getDegrees(), currentTurretAngleDeg);
    Rotation2d leadCompensation =
        odometryLookaheadTurretAngle.minus(odometryLineOfSightTurretAngle);
    Rotation2d turretAngle = odometryLookaheadTurretAngle;
    double turretAngleDeg = odometryLookaheadTurretAngleDeg;
    boolean turretVisionRawAvailable = shouldUseTurretVision();
    if (turretVisionRawAvailable) {
      double rawTurretVisionTxDeg = vision.getTargetX(turretVisionCameraIndex).getDegrees();
      if (!turretVisionLocked && turretVisionSeenLoops == 0) {
        turretVisionTxFilter = createTurretVisionTxFilter();
        filteredTurretVisionTxDeg = rawTurretVisionTxDeg;
      } else {
        filteredTurretVisionTxDeg = turretVisionTxFilter.calculate(rawTurretVisionTxDeg);
      }
      lastTurretVisionTagId = vision.getTargetTagId(turretVisionCameraIndex);
    }
    boolean turretVisionActive = updateTurretVisionLock(turretVisionRawAvailable);
    int turretVisionTagId = turretVisionActive ? lastTurretVisionTagId : -1;
    Rotation2d turretVisionTx = Rotation2d.kZero;
    Rotation2d turretVisionLineOfSightAngle = Rotation2d.kZero;
    double turretVisionLineOfSightAngleDeg = 0.0;
    double turretVisionAppliedCorrectionDeg = 0.0;
    boolean turretVisionCorrectionRejected = false;

    if (turretVisionActive) {
      turretVisionTx = Rotation2d.fromDegrees(applyTurretVisionTxDeadband(filteredTurretVisionTxDeg));
      turretVisionLineOfSightAngle =
          Rotation2d.fromDegrees(currentTurretAngleDeg)
              .minus(turretVisionTx)
              .plus(
                  Rotation2d.fromDegrees(
                      Constants.ShootOnTheMoveConstants.kTurretVisionAimOffsetDegrees));
      turretVisionLineOfSightAngleDeg =
          resolveTurretAngleToLimits(
              turretVisionLineOfSightAngle.getDegrees(), currentTurretAngleDeg);
      double turretVisionCorrectionDeg =
          shortestAngleDeltaDegrees(
              turretVisionLineOfSightAngleDeg, odometryLineOfSightTurretAngleDeg);

      if (shouldRejectVisionCorrection(
          odometryLookaheadTurretAngleDeg, turretVisionCorrectionDeg)) {
        turretVisionCorrectionRejected = true;
      } else {
        turretVisionAppliedCorrectionDeg =
            MathUtil.clamp(
                turretVisionCorrectionDeg
                    * Constants.ShootOnTheMoveConstants.kTurretVisionBlendFactor,
                -Constants.ShootOnTheMoveConstants.kTurretVisionMaxCorrectionDegrees,
                Constants.ShootOnTheMoveConstants.kTurretVisionMaxCorrectionDegrees);
        turretAngle =
            odometryLineOfSightTurretAngle
                .plus(Rotation2d.fromDegrees(turretVisionAppliedCorrectionDeg))
                .plus(leadCompensation);
        turretAngleDeg =
            resolveTurretAngleToLimits(
                turretAngle.getDegrees(), odometryLookaheadTurretAngleDeg);
      }
    }

    double hoodAngleDeg =
        Constants.ShootOnTheMoveConstants.kHoodAngleDegrees.get(clampDistance(lookaheadDistance));
    double flywheelRpm =
        Constants.ShootOnTheMoveConstants.kFlywheelSpeedRpm.get(clampDistance(lookaheadDistance));
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
        lookaheadDistance,
        turretVisionRawAvailable,
        turretVisionActive,
        turretVisionTagId,
        turretAngleDeg,
        turretAngle,
        odometryLineOfSightTurretAngleDeg,
        odometryLineOfSightTurretAngle,
        leadCompensation,
        turretVisionTx,
        filteredTurretVisionTxDeg,
        turretVisionLineOfSightAngleDeg,
        turretVisionLineOfSightAngle,
        turretVisionAppliedCorrectionDeg,
        turretVisionCorrectionRejected,
        turretVelocityDegPerSecond,
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

  private static int loopsForSeconds(double seconds) {
    return Math.max(1, (int) Math.ceil(seconds / Constants.kLoopPeriodSeconds));
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

  private static double resolveTurretAngleToLimits(double desiredAngleDeg, double referenceAngleDeg) {
    // The turret can travel exactly one turn, so the same pointing direction may have two legal
    // representations at the cable-wrap seam. Pick the legal equivalent closest to the current
    // branch so we preserve the old circular aiming behavior without commanding an unnecessary
    // full-turn unwind.
    double wrappedAngleDeg = MathUtil.inputModulus(desiredAngleDeg, kMinAngleDeg, kMaxAngleDeg);
    double resolvedAngleDeg = MathUtil.clamp(wrappedAngleDeg, kMinAngleDeg, kMaxAngleDeg);
    double alternateAngleDeg =
        wrappedAngleDeg + (wrappedAngleDeg <= referenceAngleDeg ? 360.0 : -360.0);

    if (isTurretAngleWithinLimits(alternateAngleDeg)
        && Math.abs(alternateAngleDeg - referenceAngleDeg)
            < Math.abs(resolvedAngleDeg - referenceAngleDeg)) {
      resolvedAngleDeg = alternateAngleDeg;
    }

    return MathUtil.clamp(resolvedAngleDeg, kMinAngleDeg, kMaxAngleDeg);
  }

  private static boolean isTurretAngleWithinLimits(double angleDeg) {
    return angleDeg >= kMinAngleDeg - TURRET_ANGLE_LIMIT_EPSILON_DEG
        && angleDeg <= kMaxAngleDeg + TURRET_ANGLE_LIMIT_EPSILON_DEG;
  }

  private static double applyTurretVisionTxDeadband(double txDeg) {
    return Math.abs(txDeg) < Constants.ShootOnTheMoveConstants.kTurretVisionTxDeadbandDegrees
        ? 0.0
        : txDeg;
  }

  private boolean updateTurretVisionLock(boolean hasRawTarget) {
    if (hasRawTarget) {
      turretVisionSeenLoops++;
      turretVisionMissedLoops = 0;
      if (!turretVisionLocked && turretVisionSeenLoops >= TURRET_VISION_ACQUIRE_LOOPS) {
        turretVisionLocked = true;
      }
      return turretVisionLocked;
    }

    turretVisionSeenLoops = 0;
    turretVisionLocked = false;
    turretVisionMissedLoops = 0;
    filteredTurretVisionTxDeg = 0.0;
    lastTurretVisionTagId = -1;
    return turretVisionLocked;
  }

  private boolean shouldRejectVisionCorrection(
      double odometryTurretAngleDeg, double turretVisionCorrectionDeg) {
    if (Math.abs(turretVisionCorrectionDeg)
        > Constants.ShootOnTheMoveConstants.kTurretVisionMaxOdometryDisagreementDegrees) {
      return true;
    }

    double boundaryMarginDeg =
        Constants.ShootOnTheMoveConstants.kTurretVisionBoundaryMarginDegrees;
    return pushesTowardCableLimit(turret.getAngle(), turretVisionCorrectionDeg, boundaryMarginDeg)
        || pushesTowardCableLimit(odometryTurretAngleDeg, turretVisionCorrectionDeg, boundaryMarginDeg);
  }

  private static boolean pushesTowardCableLimit(
      double angleDeg, double correctionDeg, double boundaryMarginDeg) {
    return (angleDeg >= kMaxAngleDeg - boundaryMarginDeg && correctionDeg > 0.0)
        || (angleDeg <= kMinAngleDeg + boundaryMarginDeg && correctionDeg < 0.0);
  }

  private boolean shouldUseTurretVision() {
    if (!Constants.ShootOnTheMoveConstants.kUseTurretVisionCorrection
        || vision == null
        || turretVisionCameraIndex < 0) {
      return false;
    }

    if (!vision.hasTarget(turretVisionCameraIndex)
        || vision.getTargetArea(turretVisionCameraIndex)
            < Constants.ShootOnTheMoveConstants.kTurretVisionMinTargetArea) {
      return false;
    }

    return Math.abs(drive.getRobotRelativeSpeeds().omegaRadiansPerSecond)
        <= Constants.ShootOnTheMoveConstants.kTurretVisionMaxAngularVelocityRadPerSec;
  }
}
