package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kArmCalibrationAngleDeg;
import static frc.robot.subsystems.intake.IntakeConstants.kArmCenterOfGravityDistance;
import static frc.robot.subsystems.intake.IntakeConstants.kArmGearbox;
import static frc.robot.subsystems.intake.IntakeConstants.kArmGearboxReduction;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMaxAngle;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMinAngle;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMomentOfInertiaPivot;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorStatorLimitAmps;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorSupplyLimitAmps;
import static frc.robot.subsystems.intake.IntakeConstants.kDSim;
import static frc.robot.subsystems.intake.IntakeConstants.kGSim;
import static frc.robot.subsystems.intake.IntakeConstants.kMotionMagicAccelerationDegPerSecSq;
import static frc.robot.subsystems.intake.IntakeConstants.kMotionMagicMaxVelocityDegPerSec;
import static frc.robot.subsystems.intake.IntakeConstants.kPSim;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerGearbox;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerGearboxReduction;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMomentOfInertia;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorStatorLimitAmps;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorSupplyLimitAmps;

import frc.robot.util.sim.CustomDCMotorSim;
import frc.robot.util.sim.CustomSingleJointedArmSim;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.SimulatedArena;

/** The IO sim implementation for intake hardware interacions without any hardware. */
public class IntakeIOSim implements IntakeIO {
	private CustomDCMotorSim m_rollerSim;
	private CustomSingleJointedArmSim m_armSim;

	private ProfiledPIDController m_armPid = new ProfiledPIDController(kPSim, 0, kDSim,
			new TrapezoidProfile.Constraints(kMotionMagicMaxVelocityDegPerSec, kMotionMagicAccelerationDegPerSecSq));
	private ArmFeedforward m_armFeedforward = new ArmFeedforward(0, kGSim, 0);
	private double m_closedLoopSetpoint;
	private boolean m_isArmSetpoint = false;

	/**
	 * Constructs a new IntakeIOSim.
	 */
	public IntakeIOSim() {
		m_rollerSim = new CustomDCMotorSim(
				LinearSystemId.createDCMotorSystem(kRollerGearbox, kRollerMomentOfInertia, kRollerGearboxReduction),
				kRollerGearbox, 1);
		m_rollerSim.setSupplyCurrentLimitAmps(kRollerMotorSupplyLimitAmps);
		m_rollerSim.setStatorCurrentLimitAmps(kRollerMotorStatorLimitAmps);
		// Register roller sim
		SimulatedArena.getInstance().addCustomSimulation(m_rollerSim);

		m_armSim = new CustomSingleJointedArmSim(
				LinearSystemId.createSingleJointedArmSystem(kArmGearbox, kArmMomentOfInertiaPivot,
						kArmGearboxReduction),
				kArmGearbox, kArmGearboxReduction, 2.0 * kArmCenterOfGravityDistance, Math.toRadians(kArmMinAngle),
				Math.toRadians(kArmMaxAngle), true, Math.toRadians(kArmCalibrationAngleDeg), 1);
		m_armSim.configureCurrentLimits(kArmMotorSupplyLimitAmps, kArmMotorStatorLimitAmps);
		// Register arm sim
		SimulatedArena.getInstance().addCustomSimulation(m_armSim);

		m_armPid.enableContinuousInput(-180, 180);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		if (m_isArmSetpoint) {
			double pidOutput = m_armPid.calculate(Math.toDegrees(m_armSim.getAngleRads()), m_closedLoopSetpoint);
			double ffOutput = m_armFeedforward.calculate(m_armSim.getAngleRads(), m_armPid.getSetpoint().velocity);
			m_armSim.setInputVoltage(ffOutput + pidOutput);
		}

		inputs.rollerMotorConnected = true;
		inputs.rollerAppliedVoltage = m_rollerSim.getAppliedInputVoltage();
		inputs.rollerRpm = m_rollerSim.getOutputAngularVelocityRPM();
		inputs.rollerSupplyCurrentAmps = Math.abs(m_rollerSim.getSupplyCurrentDrawAmps());
		inputs.rollerStatorCurrentAmps = Math.abs(m_rollerSim.getStatorCurrentDrawAmps());
		inputs.rollerTemperatureCelsius = 20;

		inputs.armMotorConnected = true;
		inputs.armAppliedVoltage = m_armSim.getInput(0);
		inputs.armRpm = Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec());
		inputs.armPositionDegrees = Math.toDegrees(m_armSim.getAngleRads());
		inputs.armSupplyCurrentAmps = Math.abs(m_armSim.getCurrentDrawAmps() * m_armSim.getInput(0) / 12.0);
		inputs.armStatorCurrentAmps = Math.abs(m_armSim.getCurrentDrawAmps());
		inputs.armTemperatureCelsius = 20;
	}

	@Override
	public void runRollerVolts(double volts) {
		m_rollerSim.setInputVoltage(volts);
	}

	@Override
	public void runArmVolts(double volts) {
		m_armSim.setInputVoltage(volts);
		m_isArmSetpoint = false;
	}

	@Override
	public void runArmPosition(double degrees) {
		m_closedLoopSetpoint = degrees;
		m_isArmSetpoint = true;
	}

	@Override
	public void zeroArmEncoders() {
		m_armSim.setState(kArmCalibrationAngleDeg, 0);
	}
}
