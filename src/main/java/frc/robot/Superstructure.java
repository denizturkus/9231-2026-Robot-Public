package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Superstructure.ShooterState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem; 

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

    private MotionState m_MotionState = MotionState.STATIONARY;
    private ShooterState m_ShooterState = ShooterState.IDLE;
    private IntakeState m_IntakeState = IntakeState.STOWED;
    private TargetingState m_TargetingState = TargetingState.NONE;

    private final Drive m_drive;
    private final IntakeSubsystem m_intake;
    private final HopperSubsystem m_hopper;
    private final FeederSubsystem m_feeder;
    private final TurretSubsystem m_turret;
    private final FlywheelSubsystem m_flywheel;
    private final HoodSubsystem m_hood;

    private final VisionSubsystem m_vision;
   
    private boolean m_isFeeding = false;
    private boolean m_isHopperRunning = false;

    public Superstructure(Drive drive, IntakeSubsystem intake, HopperSubsystem hopper, FeederSubsystem feeder, TurretSubsystem turret, FlywheelSubsystem flywheel, HoodSubsystem hood, VisionSubsystem vision) {
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


    public Command StartIntakingCommand() {
        return Commands.runOnce(() -> m_IntakeState = IntakeState.INTAKING)
                .andThen(Commands.parallel(
                        Commands.runOnce(m_intake::openIntake),
                        Commands.runOnce(m_intake::runIntakeRollers),
                        Commands.runOnce(() -> m_IntakeState = IntakeState.DEPLOYING)))
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
                    Commands.runOnce(m_hopper::stop)
                            .onlyIf(() -> m_isHopperRunning && m_ShooterState == ShooterState.IDLE))
                .andThen(Commands.waitUntil(m_intake::isArmNearSetpoint))
                .andThen(Commands.runOnce(() -> m_IntakeState = IntakeState.STOWED))
                .onlyIf(() -> m_IntakeState == IntakeState.INTAKING);
    }

    public Command FeedFuelCommand() {
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

    /*
    public Command TargetHubCommand() {
        return new FunctionalCommand(
                // initialize
                () -> {
                    m_ShooterState = ShooterState.LOADING;
                    m_TargetingState = TargetingState.HUB;
                },

                // execute continuously
                () -> {
                    // 1) Get vision data
                    double turretTargetDeg = m_vision.getTurretTargetDegrees();
                    double distanceMeters = m_vision.getTargetDistanceMeters();

                    // 2) Interpolate hood angle from distance
                    double hoodTargetDeg = m_hoodInterpolationTable.get(distanceMeters);
                    double flywheelTargetRPM = m_flywheelInterpolationTable.get(distanceMeters);

                    // 3) Command subsystems continuously
                    m_turret.setTurretAngle(turretTargetDeg);
                    m_hood.setHoodAngle(hoodTargetDeg);
                    m_flywheel.setVelocity(flywheelTargetRPM);

                    // 4) Update ready state
                    if (m_turret.isNearSetpoint()
                            && m_hood.isNearSetpoint()
                            && m_flywheel.isNearSetpoint()) {
                        m_ShooterState = ShooterState.READY;
                    } else {
                        m_ShooterState = ShooterState.LOADING;
                    }
                },

                // end
                interrupted -> {
                    // choose behavior you want
                    // maybe keep flywheel spinning, maybe not
                    m_turret.stop();
                    m_hood.stop();
                    m_flywheel.stop();
                    m_ShooterState = ShooterState.IDLE;
                },

                // isFinished
                () -> false
        );
    }

    
    public Command TargetAllianceCommand() {
        return new FunctionalCommand(
                // initialize
                () -> {
                    m_ShooterState = ShooterState.LOADING;
                },

                // execute continuously
                () -> {
                    // 1) Get vision data
                    double turretTargetDeg = m_vision.getTurretTargetDegrees();
                    double distanceMeters = m_vision.getTargetDistanceMeters();

                    // 2) Interpolate hood angle from distance
                    double hoodTargetDeg = m_hoodInterpolationTable.get(distanceMeters);
                    double flywheelTargetRPM = m_flywheelInterpolationTable.get(distanceMeters);

                    // 3) Command subsystems continuously
                    m_turret.setTurretAngle(turretTargetDeg);
                    m_hood.setHoodAngle(hoodTargetDeg);
                    m_flywheel.setVelocity(flywheelTargetRPM);

                    // 4) Update ready state
                    if (m_turret.isNearSetpoint()
                            && m_hood.isNearSetpoint()
                            && m_flywheel.isNearSetpoint()) {
                        m_ShooterState = ShooterState.READY;
                    } else {
                        m_ShooterState = ShooterState.LOADING;
                    }
                },

                // end
                interrupted -> {
                    // choose behavior you want
                    // maybe keep flywheel spinning, maybe not
                    m_turret.stop();
                    m_hood.stop();
                    m_flywheel.stop();
                    m_ShooterState = ShooterState.IDLE;
                },

                // isFinished
                () -> false
        );
    }

    public Command CancelTargetingCommand() {
            return Commands.parallel(Commands.runOnce(m_turret::stop), 
                Commands.runOnce(m_hood::stop), Commands.runOnce(m_flywheel::stop))
    } */

    
}