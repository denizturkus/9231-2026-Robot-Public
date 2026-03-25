package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.turretCameraIndex;
import static frc.robot.subsystems.intake.IntakeConstants.kArmClosedAngle;
import static frc.robot.subsystems.intake.IntakeConstants.kArmOpenedAngle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.commands.ShootOnTheMoveCommand.ShotSolution;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Superstructure extends SubsystemBase {
    private static final double kMovingLinearThresholdMetersPerSec = 0.05;
    private static final double kMovingAngularThresholdRadPerSec = 0.10;
    private static final double kHopperJamClearSeconds = 0.35;
    private static final Pose2d[] kEmptyPoses2d = new Pose2d[0];
    private static final Pose3d[] kEmptyPoses3d = new Pose3d[0];

    public enum MotionState {
        STATIONARY, // robot is not moving, either because the driver isn't commanding movement or because the robot is balancing in place
        MOVING; // robot is moving in any way, whether it's driver-controlled or auto-balancing
    }

    public enum IntakeState {
        STOWED, // intake arm is up and rollers are off
        STOWING, // intake arm is moving up
        DEPLOYING, // intake arm is moving down
        DEPLOYED, // intake arm is down and rollers are off
        INTAKING, // intake arm is down and rollers are running to pick up balls
    }

    public enum ShooterState {
        IDLE, // target button not pressed, turret and hood stowed, flywheel off
        LOADING, // target button pressed, turret and hood moving to position, flywheel spooling up
        READY, // flywheel at speed, hood and turret at position, ready to feed balls
        SHOOTING; // currently feeding balls into the shooter, feed button can be held for continuous shooting or tapped for single shots
    }

    public enum TargetingState {
        NONE, // not targeting anything, either because the target button isn't pressed or because the vision system can't see the target
        HUB, // currently targeting the hub to score fuel
        ALLIANCE; //currently targeting the alliance side to pass fuel
    }

    private enum HopperMode {
        STOPPED,
        FEEDING,
        JAM_CLEARING,
        MANUAL_REVERSE
    }

    private enum IntakeArmState {
        STOWED,
        STOWING,
        DEPLOYING,
        DEPLOYED
    }

    public MotionState m_MotionState = MotionState.STATIONARY;
    public ShooterState m_ShooterState = ShooterState.IDLE;
    public IntakeState m_IntakeState = IntakeState.STOWED;
    public TargetingState m_TargetingState = TargetingState.NONE;

    public final DriveSubsystem m_drive;
    public final IntakeSubsystem m_intake;
    public final HopperSubsystem m_hopper;
    public final FeederSubsystem m_feeder;
    public final TurretSubsystem m_turret;
    public final FlywheelSubsystem m_flywheel;
    public final HoodSubsystem m_hood;

    public final VisionSubsystem m_vision;

    private final LoggedNetworkNumber flywheelOpenLoopVoltageSetpoint = new LoggedNetworkNumber("LiveSetpoints/FlywheelVoltageSetpoint", 2);
    private final LoggedNetworkNumber targetingManualShotEnabledSetpoint = new LoggedNetworkNumber("LiveSetpoints/TargetingManualShotEnabled", 0);
    private final LoggedNetworkNumber turretTestAngleSetpoint = new LoggedNetworkNumber("LiveSetpoints/TurretAngleSetpoint", 30);
    private final LoggedNetworkNumber hoodTestAngleSetpoint = new LoggedNetworkNumber("LiveSetpoints/HoodAngleSetpoint", 30);
    private final LoggedNetworkNumber flywheelTestRPMSetpoint = new LoggedNetworkNumber("LiveSetpoints/FlywheelRPMSetpoint", 2500);
    private final Field2d shotField = new Field2d();

    private boolean m_isFeeding = false;
    private boolean m_isHopperRunning = false;
    private boolean m_isShootFuelActive = false;
    private boolean m_isManualHopperReverseRequested = false;
    private boolean m_isIntakeRollerRequestActive = false;
    private boolean m_isManualHoodOverrideActive = false;
    private ShotSolution m_latestShotSolution = null;
    private HopperMode m_hopperMode = HopperMode.STOPPED;
    private IntakeArmState m_intakeArmState = IntakeArmState.STOWED;
    private double m_manualHoodOverrideDeg = Double.NaN;
    private double kFlywheelOpenLoopVolts = flywheelOpenLoopVoltageSetpoint.get();
    private double kFlywheelTestRPM = flywheelTestRPMSetpoint.get();
    private double kHoodAngleTestDegree = hoodTestAngleSetpoint.get();
    private double kTurretAngleTestDegree = turretTestAngleSetpoint.get();
    private double m_hopperJamRecoveryEndTimestampSeconds = Double.NEGATIVE_INFINITY;

    public Superstructure(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, FeederSubsystem feeder, TurretSubsystem turret, FlywheelSubsystem flywheel, HoodSubsystem hood, VisionSubsystem vision) {
        m_drive = drive;
        m_intake = intake;
        m_hopper = hopper;
        m_feeder = feeder;
        m_turret = turret;
        m_flywheel = flywheel;
        m_hood = hood;
        m_vision = vision;

        SmartDashboard.putData("Superstructure/ShotField", shotField);
    }

    public MotionState getMotionState() {
        return m_MotionState;
    }

    public ShooterState getShooterState() {
        return m_ShooterState;
    }

    public IntakeState getIntakeState() {
        return m_IntakeState;
    }

    public TargetingState getTargetingState() {
        return m_TargetingState;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled() && m_isIntakeRollerRequestActive) {
            setIntakeRollerRequestActive(false);
        }

        syncIntakeArmStateToMechanism();

        kFlywheelOpenLoopVolts = flywheelOpenLoopVoltageSetpoint.get();
        kFlywheelTestRPM = flywheelTestRPMSetpoint.get();
        kHoodAngleTestDegree = hoodTestAngleSetpoint.get();
        kTurretAngleTestDegree = turretTestAngleSetpoint.get();

        var speeds = m_drive.getRobotRelativeSpeeds();
        double linearSpeedMetersPerSec =
                Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        m_MotionState = linearSpeedMetersPerSec > kMovingLinearThresholdMetersPerSec
                        || Math.abs(speeds.omegaRadiansPerSecond) > kMovingAngularThresholdRadPerSec
                ? MotionState.MOVING
                : MotionState.STATIONARY;

        boolean turretReady = m_turret.isNearSetpoint();
        boolean hoodReady = m_hood.isNearSetpoint();
        boolean flywheelReady = m_flywheel.isNearSetpoint();
        boolean shotValid = m_latestShotSolution != null && m_latestShotSolution.valid();
        boolean readyToShoot = shotValid && turretReady && hoodReady && flywheelReady;
        boolean intakeDrivingHopper = isIntakeDrivingHopper();
        boolean shooterDrivingHopper = isShooterDrivingHopper();

        updateIntakeState();
        updateHopperControl(intakeDrivingHopper, shooterDrivingHopper);

        Logger.recordOutput("Superstructure/MotionState", m_MotionState);
        Logger.recordOutput("Superstructure/IntakeState", m_IntakeState);
        Logger.recordOutput("Superstructure/IntakeArmState", m_intakeArmState.name());
        Logger.recordOutput("Superstructure/IntakeRollersRequested", m_isIntakeRollerRequestActive);
        Logger.recordOutput("Superstructure/IntakeRollersRunning", m_intake.areRollersRunning());
        Logger.recordOutput("Superstructure/ShooterState", m_ShooterState);
        Logger.recordOutput("Superstructure/TargetingState", m_TargetingState);
        Logger.recordOutput("Superstructure/IsFeeding", m_isFeeding);
        Logger.recordOutput("Superstructure/IsHopperRunning", m_isHopperRunning);
        Logger.recordOutput("Superstructure/HopperMode", m_hopperMode.name());
        Logger.recordOutput("Superstructure/HopperDrivenByIntake", intakeDrivingHopper);
        Logger.recordOutput("Superstructure/HopperDrivenByShooter", shooterDrivingHopper);
        Logger.recordOutput("Superstructure/HopperManualReverseRequested", m_isManualHopperReverseRequested);
        Logger.recordOutput("Superstructure/HopperJamClearActive", isAutomaticHopperJamRecoveryActive());
        Logger.recordOutput("Superstructure/HopperLikelyJammed", m_hopper.isLikelyJammed());
        Logger.recordOutput("Superstructure/HasShotSolution", m_latestShotSolution != null);
        Logger.recordOutput("Superstructure/LatestShotValid", shotValid);
        Logger.recordOutput("Superstructure/Ready/Turret", turretReady);
        Logger.recordOutput("Superstructure/Ready/Hood", hoodReady);
        Logger.recordOutput("Superstructure/Ready/Flywheel", flywheelReady);
        Logger.recordOutput("Superstructure/Ready/ShotValid", shotValid);
        Logger.recordOutput("Superstructure/Ready/All", readyToShoot);
        Logger.recordOutput(
                "Superstructure/StatusColors",
                new String[] {
                    colorForMotionState(),
                    colorForIntakeState(),
                    colorForTargetingState(shotValid),
                    colorForReadiness(turretReady),
                    colorForReadiness(hoodReady),
                    colorForReadiness(flywheelReady),
                    colorForReadiness(shotValid),
                    colorForReadiness(readyToShoot),
                    colorForActivity(m_isFeeding),
                });
        Logger.recordOutput("Superstructure/OverallStatusColor", colorForOverallStatus(readyToShoot));
        Logger.recordOutput(
                "Superstructure/Ready/Count",
                (turretReady ? 1 : 0) + (hoodReady ? 1 : 0) + (flywheelReady ? 1 : 0) + (shotValid ? 1 : 0));
        Logger.recordOutput("Superstructure/DriveLinearSpeedMetersPerSec", linearSpeedMetersPerSec, "meters_per_second");
        Logger.recordOutput(
                "Superstructure/DriveAngularSpeedRadPerSec",
                speeds.omegaRadiansPerSecond,
                "radians_per_second");
        Logger.recordOutput("Superstructure/TargetingManualShotEnabled", isTargetingManualShotEnabled());
        Logger.recordOutput("Superstructure/TargetingManualHoodAngleDeg", kHoodAngleTestDegree);
        Logger.recordOutput("Superstructure/TargetingManualFlywheelRpm", kFlywheelTestRPM);
        Logger.recordOutput("Superstructure/ManualHoodOverrideActive", m_isManualHoodOverrideActive);
        Logger.recordOutput("Superstructure/ManualHoodOverrideDeg", m_manualHoodOverrideDeg);

        updateShotField();
    }


    public Command ExtendIntakeArmCommand() {
        Command command =
                Commands.sequence(
                                Commands.runOnce(m_intake::openIntake, m_intake),
                                Commands.runOnce(() -> setIntakeArmState(IntakeArmState.DEPLOYING)),
                                Commands.waitUntil(m_intake::isArmNearSetpoint),
                                Commands.runOnce(() -> setIntakeArmState(IntakeArmState.DEPLOYED)))
                        .onlyIf(this::isIntakeArmRetracted);
        command.setName("ExtendIntakeArm");
        return command;
    }

    public Command RunIntakeRollersCommand() {
        Command command =
                Commands.sequence(
                                Commands.runOnce(m_intake::runIntakeRollers, m_intake),
                                Commands.runOnce(() -> setIntakeRollerRequestActive(true)))
                        .onlyIf(() -> !m_isIntakeRollerRequestActive);
        command.setName("RunIntakeRollers");
        return command;
    }

    public Command StartIntakingCommand() {
        Command command = Commands.sequence(ExtendIntakeArmCommand(), RunIntakeRollersCommand());
        command.setName("StartIntaking");
        return command;
    }

    public Command StopIntakeRollersCommand() {
        Command command =
                Commands.sequence(
                                        Commands.runOnce(m_intake::stopRollers, m_intake),
                                        Commands.runOnce(this::stopHopperUnlessShooterIsDriving),
                                        Commands.runOnce(() -> setIntakeRollerRequestActive(false)))
                        .onlyIf(() -> m_isIntakeRollerRequestActive);
        command.setName("StopIntakeRollers");
        return command;
    }

    public Command RetractIntakeArmCommand() {
        Command command =
                Commands.sequence(
                                Commands.runOnce(m_intake::closeIntake, m_intake),
                                Commands.runOnce(() -> setIntakeArmState(IntakeArmState.STOWING)),
                                Commands.waitUntil(m_intake::isArmNearSetpoint),
                                Commands.runOnce(() -> setIntakeArmState(IntakeArmState.STOWED)))
                        .onlyIf(() -> isIntakeArmExtended() && !m_isIntakeRollerRequestActive);
        command.setName("RetractIntakeArm");
        return command;
    }

    public Command StopIntakingCommand() {
        Command command = Commands.sequence(StopIntakeRollersCommand(), RetractIntakeArmCommand());
        command.setName("StopIntaking");
        return command;
    }

    public Command ShootFuelCommand() {
        Command command = Commands.run(this::updateShootFuelState, m_feeder, m_hopper, m_flywheel)
                .beforeStarting(() -> {
                    m_isShootFuelActive = true;
                    if (m_TargetingState != TargetingState.NONE) {
                        m_ShooterState = ShooterState.LOADING;
                    }
                })
                .finallyDo((interrupted) -> endShootFuel());
        command.setName("ShootFuel");
        return command;
    }

    public Command TargetHubCommand() {
        Command command = createShootOnTheMoveCommand(
                TargetingState.HUB, ShootOnTheMoveCommand.TargetingMode.HUB);
        command.setName("TargetHub");
        return command;
    }

    public Command TargetAllianceSideCommand() {
        Command command = createShootOnTheMoveCommand(
                TargetingState.ALLIANCE, ShootOnTheMoveCommand.TargetingMode.ALLIANCE_SIDE);
        command.setName("TargetAllianceSide");
        return command;
    }

    public Command CancelTargetingCommand() {
        Command command = Commands.runOnce(
                () -> {
                    m_TargetingState = TargetingState.NONE;
                    m_latestShotSolution = null;
                    m_ShooterState = ShooterState.IDLE;
                    stopHopperIfIntakeIsIdle();
                },
                m_turret,
                m_hood,
                m_flywheel)
                .onlyIf(() -> m_TargetingState != TargetingState.NONE);
        command.setName("CancelTargeting");
        return command;
    }

    private Command createShootOnTheMoveCommand(
            TargetingState targetingState, ShootOnTheMoveCommand.TargetingMode targetingMode) {
        return Commands.parallel(
                        new ShootOnTheMoveCommand(
                                m_drive,
                                m_turret,
                                m_hood,
                                targetingMode,
                                m_vision,
                                turretCameraIndex,
                                this::setLatestShotSolution,
                                this::isTargetingManualShotEnabled,
                                this::isManualHoodOverrideActive,
                                this::getRequestedTargetingHoodAngleDeg,
                                flywheelTestRPMSetpoint::get),
                        Commands.run(() -> updateShootOnTheMoveState(targetingState)))
                .beforeStarting(() -> {
                    clearManualHoodOverride();
                    m_TargetingState = targetingState;
                    m_latestShotSolution = null;
                    m_ShooterState = ShooterState.LOADING;
                })
                .finallyDo((interrupted) -> {
                    m_TargetingState = TargetingState.NONE;
                    m_latestShotSolution = null;
                    if (!m_isShootFuelActive && !m_isFeeding) {
                        m_ShooterState = ShooterState.IDLE;
                    }
                });
    }

    private void updateShootOnTheMoveState(TargetingState targetingState) {
        m_TargetingState = targetingState;
        if (m_isFeeding) {
            m_ShooterState = ShooterState.SHOOTING;
            return;
        }

        if (!m_isShootFuelActive) {
            m_ShooterState = ShooterState.LOADING;
        }
    }

    private boolean isTargetingManualShotEnabled() {
        return targetingManualShotEnabledSetpoint.get() > 0.5;
    }

    private boolean isManualHoodOverrideActive() {
        return m_isManualHoodOverrideActive;
    }

    private double getRequestedTargetingHoodAngleDeg() {
        return m_isManualHoodOverrideActive ? m_manualHoodOverrideDeg : hoodTestAngleSetpoint.get();
    }

    private void clearManualHoodOverride() {
        m_isManualHoodOverrideActive = false;
        m_manualHoodOverrideDeg = Double.NaN;
    }

    private void setManualHoodOverride(double hoodAngleDeg) {
        m_isManualHoodOverrideActive = true;
        m_manualHoodOverrideDeg = hoodAngleDeg;
        m_hood.setHoodAngle(hoodAngleDeg);
    }

    private void setLatestShotSolution(ShotSolution shotSolution) {
        m_latestShotSolution = shotSolution;
    }

    private void updateShootFuelState() {
        if (m_TargetingState == TargetingState.NONE || m_latestShotSolution == null) {
            m_flywheel.stop();
            stopFeeder();
            stopHopperIfIntakeIsIdle();
            m_ShooterState =
                    m_TargetingState == TargetingState.NONE ? ShooterState.IDLE : ShooterState.LOADING;
            return;
        }

        m_flywheel.setVelocity(m_latestShotSolution.flywheelRpm());

        if (isShotReadyToFeed()) {
            if (!m_isFeeding) {
                m_ShooterState = ShooterState.READY;
                startShooterFeed();
                return;
            }

            m_ShooterState = ShooterState.SHOOTING;
            return;
        }

        stopFeeder();
        stopHopperIfIntakeIsIdle();
        m_ShooterState = ShooterState.LOADING;
    }

    private boolean isShotReadyToFeed() {
        return m_latestShotSolution != null
                && m_latestShotSolution.valid()
                && m_turret.isNearSetpoint()
                && m_hood.isNearSetpoint()
                && m_flywheel.isNearSetpoint();
    }

    private void startShooterFeed() {
        m_feeder.feed();
        m_isFeeding = true;
    }

    private void stopFeeder() {
        m_feeder.stop();
        m_isFeeding = false;
    }

    private void stopHopperIfIntakeIsIdle() {
        if (!isIntakeDrivingHopper()) {
            clearAutomaticHopperJamRecovery();
            setHopperMode(HopperMode.STOPPED);
        }
    }

    private void stopHopperUnlessShooterIsDriving() {
        if (!isShooterDrivingHopper()) {
            clearAutomaticHopperJamRecovery();
            setHopperMode(HopperMode.STOPPED);
        }
    }

    private void endShootFuel() {
        m_isShootFuelActive = false;
        m_flywheel.stop();
        stopFeeder();
        stopHopperIfIntakeIsIdle();
        m_ShooterState =
                m_TargetingState == TargetingState.NONE ? ShooterState.IDLE : ShooterState.LOADING;
    }

    //test commands, these are not a part of the main superstructure and should not be used for competition

    public Command FlywheelTestCommand() {
        return Commands.startEnd(() -> m_flywheel.setVelocity(kFlywheelTestRPM), () -> m_flywheel.stop());
    }

    public Command FlywheelOpenLoopCommand() {
        return Commands.startEnd(() -> m_flywheel.setVoltage(kFlywheelOpenLoopVolts), () -> m_flywheel.stop());
    }

    public Command IntakeArmExtendTestCommand() {
        return Commands.runOnce(m_intake::openIntake);
    }

    public Command IntakeArmRetractTestCommand() {
        return Commands.runOnce(m_intake::closeIntake);
    }

    public Command RunIntakeRollersTestCommand() {
        return Commands.runOnce(m_intake::runIntakeRollers);
    }

    public Command StopIntakeRollersTestCommand() {
        return Commands.runOnce(m_intake::stopRollers);
    }

    public Command HopperFeederOpenLoopCommand() {
        return Commands.parallel(Commands.startEnd(m_feeder::feed, m_feeder::stop),Commands.startEnd(m_hopper::feed, m_hopper::stop));
    }

    public Command ManualReverseHopperCommand() {
        Command command =
                Commands.startEnd(
                        () -> {
                            m_isManualHopperReverseRequested = true;
                            clearAutomaticHopperJamRecovery();
                        },
                        () -> m_isManualHopperReverseRequested = false,
                        this);
        command.setName("ManualReverseHopper");
        return command;
    }

    public Command HoodManual8DegsCommand() {
        return Commands.runOnce(() -> setManualHoodOverride(8));
    }
    public Command HoodManual12DegsCommand() {
        return Commands.runOnce(() -> setManualHoodOverride(12));
    }
    public Command HoodManual16DegsCommand() {
        return Commands.runOnce(() -> setManualHoodOverride(16));
    }
    public Command HoodManual20DegsCommand() {
        return Commands.runOnce(() -> setManualHoodOverride(20));
    }
    public Command HoodManual24DegsCommand() {
        return Commands.runOnce(() -> setManualHoodOverride(24));
    }
    public Command HoodManual28DegsCommand() {
        return Commands.runOnce(() -> setManualHoodOverride(28));
    }
    public Command HoodManual32DegsCommand() {
        return Commands.runOnce(() -> setManualHoodOverride(32));
    }
    public Command HoodManual36DegsCommand() {
        return Commands.runOnce(() -> setManualHoodOverride(36));
    }

    public Command SetHoodAngleTestCommand() {
        return Commands.runOnce(() -> m_hood.setHoodAngle(kHoodAngleTestDegree));
    }

    public Command SetTurretAngleTestCommand() {
        return Commands.runOnce(() -> m_turret.setTurretAngle(kTurretAngleTestDegree));
    }

    private void updateShotField() {
        Pose2d robotPose = m_drive.getPose();
        shotField.setRobotPose(robotPose);
        logAdvantageScopeTurretTelemetry(robotPose);

        Pose2d requestedTargetPose = getRequestedTargetPose();
        Pose3d requestedTargetPose3d = getRequestedTargetPose3d();
        logRequestedTargetTelemetry(requestedTargetPose, requestedTargetPose3d);
        if (requestedTargetPose != null) {
            shotField.getObject("RequestedTarget").setPose(requestedTargetPose);
            shotField.getObject("RobotToTarget").setPoses(
                    new Pose2d(robotPose.getTranslation(), Rotation2d.kZero),
                    new Pose2d(requestedTargetPose.getTranslation(), Rotation2d.kZero));
        } else {
            shotField.getObject("RequestedTarget").setPoses();
            shotField.getObject("RobotToTarget").setPoses();
        }

        if (m_latestShotSolution == null) {
            shotField.getObject("EstimatedRobot").setPoses();
            shotField.getObject("TurretPosition").setPoses();
            shotField.getObject("PredictedTurretPosition").setPoses();
            shotField.getObject("ShotLine").setPoses();
            shotField.getObject("OdometryLine").setPoses();
            clearTrackedTargetTelemetry();
            return;
        }

        Pose2d estimatedRobotPose = m_latestShotSolution.estimatedPose();
        Pose2d turretPose = getTrackedTurretPose(m_latestShotSolution);
        Pose2d predictedTurretPose = getPredictedTurretPose(m_latestShotSolution);
        Pose2d shotTargetPose = new Pose2d(m_latestShotSolution.target(), Rotation2d.kZero);
        Pose3d turretPose3d = toTurretPose3d(turretPose);
        Pose3d predictedTurretPose3d = toTurretPose3d(predictedTurretPose);
        Pose3d trackedTargetPose3d = getTrackedTargetPose3d(m_latestShotSolution, requestedTargetPose3d);

        shotField.getObject("EstimatedRobot").setPose(estimatedRobotPose);
        shotField.getObject("TurretPosition").setPose(turretPose);
        shotField.getObject("PredictedTurretPosition").setPose(predictedTurretPose);
        shotField.getObject("ShotLine").setPoses(predictedTurretPose, shotTargetPose);
        shotField.getObject("OdometryLine").setPoses(turretPose, shotTargetPose);
        Logger.recordOutput("AdvantageScope/Targeting/TrackedTarget2d", new Pose2d[] {shotTargetPose});
        Logger.recordOutput("AdvantageScope/Targeting/TrackedTarget3d", new Pose3d[] {trackedTargetPose3d});
        Logger.recordOutput("AdvantageScope/Targeting/PredictedTurretPose2d", new Pose2d[] {predictedTurretPose});
        Logger.recordOutput("AdvantageScope/Targeting/PredictedTurretPose3d", new Pose3d[] {predictedTurretPose3d});
        Logger.recordOutput(
                "AdvantageScope/Targeting/OdometryLine2d",
                new Pose2d[] {turretPose, shotTargetPose});
        Logger.recordOutput(
                "AdvantageScope/Targeting/OdometryLine3d",
                new Pose3d[] {turretPose3d, trackedTargetPose3d});
        Logger.recordOutput(
                "AdvantageScope/Targeting/ShotLine2d",
                new Pose2d[] {predictedTurretPose, shotTargetPose});
        Logger.recordOutput(
                "AdvantageScope/Targeting/ShotLine3d",
                new Pose3d[] {predictedTurretPose3d, trackedTargetPose3d});
    }

    private Pose2d getRequestedTargetPose() {
        Translation3d target = getRequestedTargetTranslation3d();
        return target != null ? new Pose2d(target.toTranslation2d(), Rotation2d.kZero) : null;
    }

    private Translation3d getRequestedTargetTranslation3d() {
        return switch (m_TargetingState) {
            case HUB -> Constants.GenericConstants.AimPoints.getAllianceHubPosition();
            case ALLIANCE -> Constants.GenericConstants.AimPoints.getAllianceFarSidePosition();
            case NONE -> null;
        };
    }

    private Pose3d getRequestedTargetPose3d() {
        Translation3d target = getRequestedTargetTranslation3d();
        return target != null ? new Pose3d(target, new Rotation3d()) : null;
    }

    private void logAdvantageScopeTurretTelemetry(Pose2d robotPose) {
        Pose2d measuredTurretPose = getMeasuredTurretPose(robotPose);
        Logger.recordOutput("AdvantageScope/Turret/MeasuredPose2d", measuredTurretPose);
        Logger.recordOutput("AdvantageScope/Turret/MeasuredPose3d", toTurretPose3d(measuredTurretPose));
        Logger.recordOutput(
                "AdvantageScope/Turret/CameraOverride",
                new Pose3d(robotPose)
                        .plus(VisionConstants.getTurretCameraTransform(Rotation2d.fromDegrees(m_turret.getAngle()))));
    }

    private void logRequestedTargetTelemetry(Pose2d requestedTargetPose, Pose3d requestedTargetPose3d) {
        if (requestedTargetPose != null && requestedTargetPose3d != null) {
            Logger.recordOutput("AdvantageScope/Targeting/RequestedTarget2d", new Pose2d[] {requestedTargetPose});
            Logger.recordOutput("AdvantageScope/Targeting/RequestedTarget3d", new Pose3d[] {requestedTargetPose3d});
            return;
        }

        Logger.recordOutput("AdvantageScope/Targeting/RequestedTarget2d", kEmptyPoses2d);
        Logger.recordOutput("AdvantageScope/Targeting/RequestedTarget3d", kEmptyPoses3d);
    }

    private void clearTrackedTargetTelemetry() {
        Logger.recordOutput("AdvantageScope/Targeting/TrackedTarget2d", kEmptyPoses2d);
        Logger.recordOutput("AdvantageScope/Targeting/TrackedTarget3d", kEmptyPoses3d);
        Logger.recordOutput("AdvantageScope/Targeting/PredictedTurretPose2d", kEmptyPoses2d);
        Logger.recordOutput("AdvantageScope/Targeting/PredictedTurretPose3d", kEmptyPoses3d);
        Logger.recordOutput("AdvantageScope/Targeting/OdometryLine2d", kEmptyPoses2d);
        Logger.recordOutput("AdvantageScope/Targeting/OdometryLine3d", kEmptyPoses3d);
        Logger.recordOutput("AdvantageScope/Targeting/ShotLine2d", kEmptyPoses2d);
        Logger.recordOutput("AdvantageScope/Targeting/ShotLine3d", kEmptyPoses3d);
    }

    private Pose2d getMeasuredTurretPose(Pose2d robotPose) {
        Pose2d turretBasePose = robotPose.transformBy(GeomUtil.toTransform2d(Constants.robotToTurret));
        Rotation2d fieldRelativeTurretRotation =
                turretBasePose.getRotation().plus(Rotation2d.fromDegrees(m_turret.getAngle()));
        return new Pose2d(turretBasePose.getTranslation(), fieldRelativeTurretRotation);
    }

    private Pose2d getTrackedTurretPose(ShotSolution shotSolution) {
        Pose2d turretBasePose = shotSolution.estimatedPose().transformBy(GeomUtil.toTransform2d(Constants.robotToTurret));
        Rotation2d fieldRelativeTurretRotation = turretBasePose.getRotation().plus(shotSolution.turretAngle());
        return new Pose2d(shotSolution.turretPosition(), fieldRelativeTurretRotation);
    }

    private Pose2d getPredictedTurretPose(ShotSolution shotSolution) {
        Pose2d turretPose = getTrackedTurretPose(shotSolution);
        return new Pose2d(shotSolution.lookaheadTurretPosition(), turretPose.getRotation());
    }

    private Pose3d getTrackedTargetPose3d(ShotSolution shotSolution, Pose3d requestedTargetPose3d) {
        if (requestedTargetPose3d != null) {
            return requestedTargetPose3d;
        }
        return new Pose3d(
                new Translation3d(shotSolution.target().getX(), shotSolution.target().getY(), 0.0),
                new Rotation3d());
    }

    private Pose3d toTurretPose3d(Pose2d turretPose) {
        return new Pose3d(turretPose)
                .plus(new Transform3d(0.0, 0.0, Constants.robotToTurret.getZ(), new Rotation3d()));
    }

    private String colorForMotionState() {
        return m_MotionState == MotionState.MOVING ? "#20C997" : "#495057";
    }

    private String colorForIntakeState() {
        return switch (m_IntakeState) {
            case INTAKING -> "#40C057";
            case DEPLOYED -> "#15AABF";
            case DEPLOYING, STOWING -> "#FFA94D";
            case STOWED -> "#495057";
        };
    }

    private String colorForTargetingState(boolean shotValid) {
        if (m_TargetingState == TargetingState.NONE) {
            return "#495057";
        }
        return shotValid ? "#339AF0" : "#FFA94D";
    }

    private static String colorForReadiness(boolean ready) {
        return ready ? "#40C057" : "#495057";
    }

    private static String colorForActivity(boolean active) {
        return active ? "#FF922B" : "#495057";
    }

    private String colorForOverallStatus(boolean readyToShoot) {
        if (m_isFeeding) {
            return "#FF922B";
        }
        if (readyToShoot) {
            return "#40C057";
        }
        if (m_TargetingState != TargetingState.NONE) {
            return "#339AF0";
        }
        if (m_IntakeState == IntakeState.INTAKING) {
            return "#20C997";
        }
        if (m_IntakeState == IntakeState.DEPLOYED) {
            return "#15AABF";
        }
        return "#495057";
    }

    private void setIntakeArmState(IntakeArmState intakeArmState) {
        m_intakeArmState = intakeArmState;
        updateIntakeState();
    }

    private void setIntakeRollerRequestActive(boolean intakeRollerRequestActive) {
        m_isIntakeRollerRequestActive = intakeRollerRequestActive;
        updateIntakeState();
    }

    private void syncIntakeArmStateToMechanism() {
        if (m_intake.isArmNearAngle(kArmOpenedAngle)) {
            m_intakeArmState = IntakeArmState.DEPLOYED;
            return;
        }

        if (m_intake.isArmNearAngle(kArmClosedAngle)) {
            m_intakeArmState = IntakeArmState.STOWED;
            return;
        }

        double latestSetpoint = m_intake.getLatestArmSetpoint();
        if (isNearAngle(latestSetpoint, kArmOpenedAngle, 0.1)) {
            m_intakeArmState = IntakeArmState.DEPLOYING;
        } else if (isNearAngle(latestSetpoint, kArmClosedAngle, 0.1)) {
            m_intakeArmState = IntakeArmState.STOWING;
        }
    }

    private void updateIntakeState() {
        m_IntakeState =
                switch (m_intakeArmState) {
                    case STOWED -> IntakeState.STOWED;
                    case STOWING -> IntakeState.STOWING;
                    case DEPLOYING -> IntakeState.DEPLOYING;
                    case DEPLOYED -> m_isIntakeRollerRequestActive ? IntakeState.INTAKING : IntakeState.DEPLOYED;
                };
    }

    private boolean isIntakeArmRetracted() {
        return m_intakeArmState == IntakeArmState.STOWED || m_intakeArmState == IntakeArmState.STOWING;
    }

    private boolean isIntakeArmExtended() {
        return m_intakeArmState == IntakeArmState.DEPLOYING || m_intakeArmState == IntakeArmState.DEPLOYED;
    }

    private boolean isIntakeDrivingHopper() {
        return m_isIntakeRollerRequestActive && !DriverStation.isDisabled();
    }

    private boolean isShooterDrivingHopper() {
        return m_feeder.isFeedingForward() && m_flywheel.isRunning();
    }

    private static boolean isNearAngle(double angleA, double angleB, double toleranceDeg) {
        return Math.abs(MathUtil.inputModulus(angleA - angleB, -180, 180)) <= toleranceDeg;
    }

    private void updateHopperControl(boolean intakeDrivingHopper, boolean shooterDrivingHopper) {
        boolean shouldFeedForward = intakeDrivingHopper || shooterDrivingHopper;
        boolean hopperLikelyJammed = shouldFeedForward && m_hopper.isLikelyJammed();

        if (!shouldFeedForward) {
            clearAutomaticHopperJamRecovery();
        } else if (hopperLikelyJammed
                && !isAutomaticHopperJamRecoveryActive()
                && !m_isManualHopperReverseRequested) {
            requestAutomaticHopperJamRecovery();
        }

        if (m_isManualHopperReverseRequested) {
            clearAutomaticHopperJamRecovery();
            setHopperMode(HopperMode.MANUAL_REVERSE);
            return;
        }

        if (isAutomaticHopperJamRecoveryActive()) {
            setHopperMode(HopperMode.JAM_CLEARING);
            return;
        }

        if (shouldFeedForward) {
            setHopperMode(HopperMode.FEEDING);
            return;
        }

        setHopperMode(HopperMode.STOPPED);
    }

    private void requestAutomaticHopperJamRecovery() {
        m_hopperJamRecoveryEndTimestampSeconds =
                Timer.getFPGATimestamp() + kHopperJamClearSeconds;
    }

    private void clearAutomaticHopperJamRecovery() {
        m_hopperJamRecoveryEndTimestampSeconds = Double.NEGATIVE_INFINITY;
    }

    private boolean isAutomaticHopperJamRecoveryActive() {
        return Timer.getFPGATimestamp() < m_hopperJamRecoveryEndTimestampSeconds;
    }

    private void setHopperMode(HopperMode hopperMode) {
        if (m_hopperMode == hopperMode) {
            m_isHopperRunning = hopperMode != HopperMode.STOPPED;
            return;
        }

        m_hopperMode = hopperMode;
        switch (hopperMode) {
            case FEEDING -> m_hopper.feed();
            case JAM_CLEARING, MANUAL_REVERSE -> m_hopper.reverse();
            case STOPPED -> m_hopper.stop();
        }
        m_isHopperRunning = hopperMode != HopperMode.STOPPED;
    }
}
