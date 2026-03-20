// Copyright 2021-2024 FRC 6328
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

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.indexer.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.feeder.FeederSubsystem;
import frc.robot.subsystems.indexer.feeder.FeederIOTalonFX;
import frc.robot.subsystems.indexer.feeder.FeederIO;
import frc.robot.subsystems.indexer.feeder.FeederIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFX;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
    //drivetrain
    private final DriveSubsystem drive;
    private SwerveDriveSimulation driveSimulation = null;

    //vision
    private final VisionSubsystem vision;
    
    //intake
    private final IntakeSubsystem intake;

    //indexer
    private final HopperSubsystem hopper;
    private final FeederSubsystem feeder;

    //shooter
    private final FlywheelSubsystem flywheel;
    private final TurretSubsystem turret;
    private final HoodSubsystem hood;

    private final Superstructure superstructure;

    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final CommandXboxController musicController = new CommandXboxController(2);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser; 
    public static Alliance currentAlliance = Alliance.Red;
    public final LoggedDashboardChooser<Alliance> m_allianceChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new DriveSubsystem(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (pose) -> {});
                intake = new IntakeSubsystem(new IntakeIOTalonFX());
                hopper = new HopperSubsystem();
                feeder = new FeederSubsystem(new FeederIOTalonFX());
                flywheel = new FlywheelSubsystem(new FlywheelIOTalonFX());
                turret = new TurretSubsystem(new TurretIOTalonFX());
                hood = new HoodSubsystem(new HoodIOTalonFX());
                this.vision = new VisionSubsystem(
                        drive,
                        new VisionIOLimelight(
                                VisionConstants.camera0Name,
                                drive::getRotation,
                                () -> Math.toDegrees(drive.getRobotRelativeSpeeds().omegaRadiansPerSecond),
                                () -> VisionConstants.robotToCamera0),
                        new VisionIOLimelight(
                                VisionConstants.camera1Name,
                                drive::getRotation,
                                () -> Math.toDegrees(drive.getRobotRelativeSpeeds().omegaRadiansPerSecond),
                                () -> VisionConstants.getTurretCameraTransform(Rotation2d.fromDegrees(turret.getAngle()))));
                superstructure = new Superstructure(drive, intake, hopper, feeder, turret, flywheel, hood,vision);

                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                driveSimulation = new SwerveDriveSimulation(DriveSubsystem.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new DriveSubsystem(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight, driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);
                intake = new IntakeSubsystem(new IntakeIOSim());
                hopper = new HopperSubsystem();
                feeder = new FeederSubsystem(new FeederIOSim());
                flywheel = new FlywheelSubsystem(new FlywheelIOSim());
                turret = new TurretSubsystem(new TurretIOSim());
                hood = new HoodSubsystem(new HoodIOSim());
                vision = new VisionSubsystem(
                        drive,
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera0Name,
                                VisionConstants.robotToCamera0,
                                driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera1Name,
                                () -> VisionConstants.getTurretCameraTransform(Rotation2d.fromDegrees(turret.getAngle())),
                                driveSimulation::getSimulatedDriveTrainPose));
                superstructure = new Superstructure(drive, intake, hopper, feeder, turret, flywheel, hood,vision);

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new DriveSubsystem(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (pose) -> {});
                vision = new VisionSubsystem(drive, new VisionIO() {}, new VisionIO() {});
                intake = new IntakeSubsystem(new IntakeIO() {});
                hopper = new HopperSubsystem();
                feeder = new FeederSubsystem(new FeederIO() {});
                flywheel = new FlywheelSubsystem(new FlywheelIO() {});
                turret = new TurretSubsystem(new TurretIO() {});
                hood = new HoodSubsystem(new HoodIO() {});
                superstructure = new Superstructure(drive, intake, hopper, feeder, turret, flywheel, hood, vision);

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        m_allianceChooser = new LoggedDashboardChooser<>("Alliance Chooser");

        m_allianceChooser.addDefaultOption("Red Alliance", Alliance.Red);
        m_allianceChooser.addOption("Blue Alliance", Alliance.Blue);

        // register named commands for auto path following
        registerNamedCommands();

        // Configure the button bindings
        configureButtonBindings();
        
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("StartIntaking", superstructure.StartIntakingCommand());
        NamedCommands.registerCommand("StopIntaking", superstructure.StopIntakingCommand());
        NamedCommands.registerCommand("ShootFuel", superstructure.ShootFuelCommand());
        NamedCommands.registerCommand("TargetHub", superstructure.TargetHubCommand());
        NamedCommands.registerCommand("TargetAllianceSide", superstructure.TargetAllianceSideCommand());
        NamedCommands.registerCommand("CancelTargeting",superstructure.CancelTargetingCommand());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> -driverController.getRightX()));

        // Lock to 0° when A button is held
        driverController
                .a()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(
                        driveSimulation.getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
                // simulation
                : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        driverController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

      //operatorController.a().onTrue(superstructure.StartIntakingCommand());
      //operatorController.b().onTrue(superstructure.StopIntakingCommand());
        operatorController.x().whileTrue(superstructure.ShootFuelCommand());
        operatorController.rightTrigger().whileTrue(superstructure.FlywheelOpenLoopCommand());
        operatorController.rightBumper().onTrue(superstructure.TargetHubCommand());
        operatorController.leftBumper().onTrue(superstructure.TargetAllianceSideCommand());

    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());

    }
}
