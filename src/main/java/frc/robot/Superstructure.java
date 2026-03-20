package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.turretCameraIndex;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
    public enum MotionState {
        STATIONARY, // robot is not moving, either because the driver isn't commanding movement or because the robot is balancing in place
        MOVING; // robot is moving in any way, whether it's driver-controlled or auto-balancing
    }

    public enum IntakeState {
        STOWED, // intake arm is up and rollers are off
        STOWING, // intake arm is moving up, rollers are turning off
        DEPLOYING, // intake arm is moving down, rollers are turning on
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

    private boolean m_isFeeding = false;
    private boolean m_isHopperRunning = false;
    private double kFlywheel3000RPM = 3000;

    public Superstructure(DriveSubsystem drive, IntakeSubsystem intake, HopperSubsystem hopper, FeederSubsystem feeder, TurretSubsystem turret, FlywheelSubsystem flywheel, HoodSubsystem hood, VisionSubsystem vision) {
        m_drive = drive;
        m_intake = intake;
        m_hopper = hopper;
        m_feeder = feeder;
        m_turret = turret;
        m_flywheel = flywheel;
        m_hood = hood;
        m_vision = vision;
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


    public Command StartIntakingCommand() {
        return Commands.parallel(
                        Commands.runOnce(m_intake::openIntake),
                        Commands.runOnce(m_intake::runIntakeRollers),
                        Commands.runOnce(() -> m_IntakeState = IntakeState.DEPLOYING))
                .andThen(
                        Commands.runOnce(m_hopper::feed).onlyIf(() -> !m_isHopperRunning))
                .andThen(Commands.runOnce(() -> m_isHopperRunning = true))
                .andThen(Commands.waitUntil(m_intake::isArmNearSetpoint))
                .andThen(Commands.runOnce(() -> m_IntakeState = IntakeState.INTAKING))
                .onlyIf(() -> m_IntakeState == IntakeState.STOWED);
    }

    public Command StopIntakingCommand() {
        return Commands.parallel(
                    Commands.runOnce(m_intake::stopRollers),
                    Commands.runOnce(m_intake::closeIntake),
                    Commands.runOnce(() -> m_IntakeState = IntakeState.STOWING))
                .andThen(
                    Commands.runOnce(m_hopper::stop).andThen(Commands.runOnce(() -> m_isHopperRunning = false))
                            .onlyIf(() -> m_isHopperRunning && m_ShooterState == ShooterState.IDLE))
                .andThen(Commands.waitUntil(m_intake::isArmNearSetpoint))
                .andThen(Commands.runOnce(() -> m_IntakeState = IntakeState.STOWED))
                .onlyIf(() -> m_IntakeState == IntakeState.INTAKING);
    }

    public Command FlywheelOpenLoopCommand() {
        
        return Commands.startEnd(() -> m_flywheel.setVelocity(kFlywheel3000RPM), () -> m_flywheel.stop(), m_flywheel);
    }

    public Command ShootFuelCommand() {
        return Commands.startEnd(
                () -> {
                    m_feeder.feed();
                    m_isFeeding = true;
                    m_ShooterState = ShooterState.SHOOTING;
                },
                () -> {
                    m_feeder.stop();
                    m_isFeeding = false;
                    m_ShooterState = ShooterState.READY;
                },
                m_feeder
        ).onlyIf(() -> m_ShooterState == ShooterState.READY);
    }

    public Command TargetHubCommand() {
        Command command = createShootOnTheMoveCommand(
                TargetingState.HUB, ShootOnTheMoveCommand.hubTargetSupplier())
                .andThen(Commands.runOnce(m_hopper::feed).onlyIf(() -> !m_isHopperRunning));
        command.setName("TargetHub");
        return command;
    }

    public Command TargetAllianceSideCommand() {
        Command command = createShootOnTheMoveCommand(
                TargetingState.ALLIANCE, ShootOnTheMoveCommand.allianceSideTargetSupplier());
        command.setName("TargetAllianceSide");
        return command;
    }

    public Command CancelTargetingCommand() {
        Command command = Commands.runOnce(
                () -> {
                    m_TargetingState = TargetingState.NONE;
                    m_ShooterState = ShooterState.IDLE;
                    if (m_IntakeState == IntakeState.STOWED) {
                        Commands.runOnce(m_hopper::stop).schedule();
                        m_isHopperRunning = false;
                    }
                },
                m_turret,
                m_hood,
                m_flywheel)
                .onlyIf(() -> m_TargetingState != TargetingState.NONE);
        command.setName("CancelTargeting");
        return command;
    }

    private Command createShootOnTheMoveCommand(
            TargetingState targetingState, Supplier<Translation2d> targetSupplier) {
        return Commands.parallel(
                        new ShootOnTheMoveCommand(
                                m_drive,
                                m_turret,
                                m_hood,
                                m_flywheel,
                                targetSupplier,
                                m_vision,
                                turretCameraIndex),
                        Commands.run(() -> updateShootOnTheMoveState(targetingState)))
                .beforeStarting(() -> {
                    m_TargetingState = targetingState;
                    m_ShooterState = ShooterState.LOADING;
                })
                .finallyDo((interrupted) -> {
                    m_TargetingState = TargetingState.NONE;
                    if (!m_isFeeding) {
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

        if (m_turret.isNearSetpoint() && m_hood.isNearSetpoint() && m_flywheel.isNearSetpoint()) {
            m_ShooterState = ShooterState.READY;
        } else {
            m_ShooterState = ShooterState.LOADING;
        }
    }

}
