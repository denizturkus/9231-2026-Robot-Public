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

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
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
    private final Orchestra orchestra = new Orchestra();
    private boolean musicLoaded = false;

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
                flywheel = new FlywheelSubsystem();
                turret = new TurretSubsystem();
                hood = new HoodSubsystem();
                this.vision = new VisionSubsystem(
                        drive,
                        new VisionIOLimelight(
                                VisionConstants.camera0Name,
                                drive::getFieldRotation,
                                drive::getGyroYawRateDegPerSec,
                                drive::isGyroConnected,
                                () -> VisionConstants.robotToCamera0,
                                VisionConstants.getSimpleTargetAllowedTagIds(VisionConstants.chassisCameraIndex),
                                VisionConstants.isCameraMegaTag1Enabled(VisionConstants.chassisCameraIndex),
                                VisionConstants.isCameraMegaTag2Enabled(VisionConstants.chassisCameraIndex)),
                        new VisionIOLimelight(
                                VisionConstants.camera1Name,
                                drive::getFieldRotation,
                                drive::getGyroYawRateDegPerSec,
                                () -> VisionConstants.getTurretCameraTransform(Rotation2d.fromDegrees(turret.getAngle())),
                                VisionConstants.getSimpleTargetAllowedTagIds(VisionConstants.turretCameraIndex),
                                VisionConstants.isCameraMegaTag1Enabled(VisionConstants.turretCameraIndex),
                                VisionConstants.isCameraMegaTag2Enabled(VisionConstants.turretCameraIndex)));
                superstructure = new Superstructure(drive, intake, hopper, feeder, turret, flywheel, hood, vision);

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
                flywheel = new FlywheelSubsystem();
                turret = new TurretSubsystem();
                hood = new HoodSubsystem();
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
                flywheel = new FlywheelSubsystem();
                turret = new TurretSubsystem();
                hood = new HoodSubsystem();
                superstructure = new Superstructure(drive, intake, hopper, feeder, turret, flywheel, hood, vision);

                break;
        }

        initializeRobotState();

        // Register named commands before building PathPlanner autos.
        registerNamedCommands();

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
        autoChooser.addOption(
                "Flywheel SysId (Quasistatic Forward)",
                flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Flywheel SysId (Quasistatic Reverse)",
                flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Flywheel SysId (Dynamic Forward)",
                flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Flywheel SysId (Dynamic Reverse)",
                flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        m_allianceChooser = new LoggedDashboardChooser<>("Alliance Chooser");

        m_allianceChooser.addDefaultOption("Red Alliance", Alliance.Red);
        m_allianceChooser.addOption("Blue Alliance", Alliance.Blue);
        syncAllianceSelection();

        // Configure the button bindings
        configureButtonBindings();
        
    }

    private void initializeRobotState() {
        switch (Constants.currentMode) {
            case REAL:
                intake.zeroEncoders();
                turret.zeroEncoders();
                hood.zeroEncoders();
                drive.setPose(Constants.kStartingPose);
                break;
            case SIM:
                intake.zeroEncoders();
                turret.zeroEncoders();
                hood.zeroEncoders();
                drive.setPose(driveSimulation.getSimulatedDriveTrainPose());
                break;
            default:
                drive.setPose(Constants.kStartingPose);
                break;
        }
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("ExtendIntakeArm", superstructure.ExtendIntakeArmCommand());
        NamedCommands.registerCommand("RunIntakeRollers", superstructure.RunIntakeRollersCommand());
        NamedCommands.registerCommand("RetractIntakeArm", superstructure.RetractIntakeArmCommand());
        NamedCommands.registerCommand("StopIntakeRollers", superstructure.StopIntakeRollersCommand());
        NamedCommands.registerCommand("StartIntaking", superstructure.StartIntakingCommand());
        NamedCommands.registerCommand("StopIntaking", superstructure.StopIntakingCommand());
        NamedCommands.registerCommand("ShootFuel", superstructure.ShootFuelCommand());
        NamedCommands.registerCommand("TargetHub", superstructure.TargetHubCommand());
        NamedCommands.registerCommand("StationaryHubShootAuto", superstructure.StationaryHubShootAutoCommand());
        NamedCommands.registerCommand("TargetAllianceSide", superstructure.TargetAllianceSideCommand());
        NamedCommands.registerCommand("CancelTargeting",superstructure.CancelTargetingCommand());
    }

    private void syncAllianceSelection() {
        Alliance selectedAlliance = m_allianceChooser.get();
        if (selectedAlliance != null) {
            currentAlliance = selectedAlliance;
        }
    }

    public void enabledInit() {
        syncAllianceSelection();
    }

    public void disabledPeriodic() {
        syncAllianceSelection();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // DRIVE-RELATED CONTROLS ---------------------------------

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
                : drive::syncGyroToEstimatedPose;
        driverController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // OPERATIVE CONTROLS ------------------------------------

        // intake
        operatorController.a().onTrue(superstructure.ExtendIntakeArmCommand());
        operatorController.leftStick().onTrue(superstructure.RunIntakeRollersCommand());
        operatorController.rightStick().onTrue(superstructure.StopIntakeRollersCommand());
        operatorController.b().onTrue(superstructure.RetractIntakeArmCommand());

        // feed balls into the shooter
        operatorController.x().whileTrue(superstructure.ShootFuelCommand());

        operatorController.y().whileTrue(superstructure.ManualReverseHopperCommand());


        
        // automated targeting
        operatorController.rightBumper().onTrue(
                Commands.runOnce(() -> CommandScheduler.getInstance().schedule(superstructure.TargetHubCommand())));
        operatorController.leftBumper().onTrue(
                Commands.runOnce(() -> CommandScheduler.getInstance().schedule(superstructure.TargetAllianceSideCommand())));
        operatorController.leftTrigger().onTrue(superstructure.CancelTargetingCommand());

        // sets hood angles manually (degrees)
        operatorController.povDown().onTrue(superstructure.HoodManual8DegsCommand());
        operatorController.povDownRight().onTrue(superstructure.HoodManual12DegsCommand());
        operatorController.povRight().onTrue(superstructure.HoodManual16DegsCommand());
        operatorController.povUpRight().onTrue(superstructure.HoodManual20DegsCommand());
        operatorController.povUp().onTrue(superstructure.HoodManual24DegsCommand());
        operatorController.povUpLeft().onTrue(superstructure.HoodManual28DegsCommand());
        operatorController.povLeft().onTrue(superstructure.HoodManual32DegsCommand());
        operatorController.povDownLeft().onTrue(superstructure.HoodManual36DegsCommand());

        // zeroes hood and turret encoders and gets current positions as their zeroes
        operatorController.back().onTrue(Commands.parallel(Commands.runOnce(turret::zeroEncoders), Commands.runOnce(hood::zeroEncoders)));

                
       // operatorController.rightBumper().onTrue(superstructure.SetTurretAngleTestCommand());
        //operatorController.leftBumper().onTrue(superstructure.SetHoodAngleTestCommand()); 

        // some music in case robot fails to motivate people
        musicController.leftStick().onTrue(Commands.runOnce(this::configureOrchestra).ignoringDisable(true));
        musicController.start().onTrue(Commands.runOnce(this::startMusic).ignoringDisable(true));
        musicController.a().onTrue(Commands.runOnce(() -> loadMusic("IAMMUSIC/rickroll.chrp")).ignoringDisable(true));
        musicController.b().onTrue(Commands.runOnce(() -> loadMusic("IAMMUSIC/bloodytears.chrp")).ignoringDisable(true));
        musicController.x().onTrue(Commands.runOnce(() -> loadMusic("IAMMUSIC/bohemianrhapsody.chrp")).ignoringDisable(true));
        musicController.y().onTrue(Commands.runOnce(() -> loadMusic("IAMMUSIC/hesapirate.chrp")).ignoringDisable(true));
        musicController.rightBumper().onTrue(Commands.runOnce(() -> loadMusic("IAMMUSIC/thunderstruck.chrp")).ignoringDisable(true));
        musicController.leftBumper().onTrue(Commands.runOnce(() -> loadMusic("IAMMUSIC/stairway.chrp")).ignoringDisable(true));
        musicController.rightTrigger().onTrue(Commands.runOnce(() -> loadMusic("IAMMUSIC/evangelion.chrp")).ignoringDisable(true));
        musicController.leftTrigger().onTrue(Commands.runOnce(() -> loadMusic("IAMMUSIC/fingersnap.chrp")).ignoringDisable(true));
        musicController.back().onTrue(Commands.runOnce(this::stopMusic, drive).ignoringDisable(true));

    }

    private void configureOrchestra() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            return;
        }

        orchestra.stop();
        orchestra.clearInstruments();
        drive.getOrchestraDevices().forEach(orchestra::addInstrument);
        flywheel.getOrchestraDevices().forEach(orchestra::addInstrument);
        intake.getOrchestraDevices().forEach(orchestra::addInstrument);
    }

    private void loadMusic(String song) {
        String path = Filesystem.getDeployDirectory().toPath().resolve(song).toString();
        musicLoaded = orchestra.loadMusic(path).isOK();
        if (!musicLoaded) {
            DriverStation.reportError("Failed to load music: " + path, false);
        }
    }

    private void startMusic() {
        if (!musicLoaded) {
            return;
        }

        drive.stop();
        if (!orchestra.play().isOK()) {
            DriverStation.reportError("Failed to play music.", false);
        }
    }

    private void stopMusic() {
        orchestra.stop();
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
