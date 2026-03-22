package frc.robot.commands;

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
  private static final int VELOCITY_FILTER_SAMPLES =
      Math.max(
          1,
          (int)
              Math.round(
                  Constants.ShootOnTheMoveConstants.kVelocityFilterWindowSeconds
                      / Constants.kLoopPeriodSeconds));

  private final DriveSubsystem drive;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final FlywheelSubsystem flywheel;
  private final Supplier<Translation2d> targetSupplier;
  private final VisionSubsystem vision;
  private final int turretVisionCameraIndex;
  private final Consumer<ShotSolution> shotSolutionConsumer;

  private final LinearFilter turretVelocityFilter =
      LinearFilter.movingAverage(VELOCITY_FILTER_SAMPLES);
  private final LinearFilter hoodVelocityFilter =
      LinearFilter.movingAverage(VELOCITY_FILTER_SAMPLES);

  private Rotation2d lastTurretAngle = Rotation2d.kZero;
  private double lastHoodAngleDeg = 0.0;
  private ShotSolution latestSolution;

  public record ShotSolution(
      boolean valid,
      Translation2d target,
      Pose2d estimatedPose,
      Translation2d turretPosition,
      Translation2d lookaheadTurretPosition,
      double distanceMeters,
      boolean turretVisionActive,
      Rotation2d turretAngle,
      Rotation2d odometryTurretAngle,
      Rotation2d leadCompensation,
      Rotation2d turretVisionTx,
      Rotation2d turretVisionLineOfSightAngle,
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
    lastTurretAngle = Rotation2d.fromDegrees(turret.getAngle());
    lastHoodAngleDeg = hood.getAngle();
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

    turret.setTurretAngle(latestSolution.turretAngle().getDegrees());
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
    Logger.recordOutput("ShootOnTheMove/TurretVisionActive", latestSolution.turretVisionActive());
    Logger.recordOutput("ShootOnTheMove/TurretAngleDeg", latestSolution.turretAngle().getDegrees());
    Logger.recordOutput(
        "ShootOnTheMove/OdometryTurretAngleDeg", latestSolution.odometryTurretAngle().getDegrees());
    Logger.recordOutput(
        "ShootOnTheMove/TurretLeadCompensationDeg",
        latestSolution.leadCompensation().getDegrees());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionTxDeg", latestSolution.turretVisionTx().getDegrees());
    Logger.recordOutput(
        "ShootOnTheMove/TurretVisionLineOfSightAngleDeg",
        latestSolution.turretVisionLineOfSightAngle().getDegrees());
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

    Rotation2d turretAngle =
        target.minus(lookaheadTurretPosition).getAngle().minus(estimatedPose.getRotation());
    Rotation2d odometryLineOfSightTurretAngle =
        target.minus(turretPosition).getAngle().minus(estimatedPose.getRotation());
    Rotation2d leadCompensation = turretAngle.minus(odometryLineOfSightTurretAngle);
    Rotation2d turretVisionTx = Rotation2d.kZero;
    Rotation2d turretVisionLineOfSightAngle = Rotation2d.kZero;
    boolean turretVisionActive = shouldUseTurretVision();

    if (turretVisionActive) {
      turretVisionTx = vision.getTargetX(turretVisionCameraIndex);
      turretVisionLineOfSightAngle =
          Rotation2d.fromDegrees(turret.getAngle())
              .minus(turretVisionTx)
              .plus(
                  Rotation2d.fromDegrees(
                      Constants.ShootOnTheMoveConstants.kTurretVisionAimOffsetDegrees));
      turretAngle = turretVisionLineOfSightAngle.plus(leadCompensation);
    }

    double hoodAngleDeg =
        Constants.ShootOnTheMoveConstants.kHoodAngleDegrees.get(clampDistance(lookaheadDistance));
    double flywheelRpm =
        Constants.ShootOnTheMoveConstants.kFlywheelSpeedRpm.get(clampDistance(lookaheadDistance));
    double turretVelocityDegPerSecond =
        turretVelocityFilter.calculate(
            turretAngle.minus(lastTurretAngle).getDegrees() / Constants.kLoopPeriodSeconds);
    double hoodVelocityDegPerSecond =
        hoodVelocityFilter.calculate((hoodAngleDeg - lastHoodAngleDeg) / Constants.kLoopPeriodSeconds);

    lastTurretAngle = turretAngle;
    lastHoodAngleDeg = hoodAngleDeg;

    return new ShotSolution(
        lookaheadDistance >= Constants.ShootOnTheMoveConstants.kMinDistanceMeters
            && lookaheadDistance <= Constants.ShootOnTheMoveConstants.kMaxDistanceMeters,
        target,
        estimatedPose,
        turretPosition,
        lookaheadTurretPosition,
        lookaheadDistance,
        turretVisionActive,
        turretAngle,
        target.minus(lookaheadTurretPosition).getAngle().minus(estimatedPose.getRotation()),
        leadCompensation,
        turretVisionTx,
        turretVisionLineOfSightAngle,
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

  private boolean shouldUseTurretVision() {
    return Constants.ShootOnTheMoveConstants.kUseTurretVisionCorrection
        && vision != null
        && turretVisionCameraIndex >= 0
        && vision.hasTarget(turretVisionCameraIndex);
  }
}
