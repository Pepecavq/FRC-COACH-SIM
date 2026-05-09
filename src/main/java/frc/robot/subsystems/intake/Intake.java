package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.IntakeSimulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;
import frc.robot.Robot;
import frc.robot.subsystems.drive.GeneratedDrivetrain;

public class Intake extends ServoMotorSubsystem<MotorIOTalonFX> {

	private final StructPublisher<Pose3d> intakePublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Intake", Pose3d.struct)
			.publish();

	public static final Setpoint STOW =
			Setpoint.withMotionMagicSetpoint(IntakeConstants.converter.toAngle(IntakeConstants.kStowPosition));
	public static final Setpoint DEPLOY =
			Setpoint.withMotionMagicSetpoint(IntakeConstants.converter.toAngle(IntakeConstants.kDeployPosition));

	public static final Intake mInstance = new Intake();

	public IntakeSimulation intakeSimulation;

	private Intake() {
		super(
				IntakeConstants.getMotorIO(),
				"Intake",
				IntakeConstants.converter.toAngle(IntakeConstants.kEpsilonThreshold),
				IntakeConstants.getServoConfig());
		setCurrentPosition(IntakeConstants.converter.toAngle(IntakeConstants.kStowPosition));
		applySetpoint(STOW);

		if (Robot.isSimulation()) {
			intakeSimulation = IntakeSimulation.InTheFrameIntake(
					"Fuel",
					GeneratedDrivetrain.mapleSimSwerveDrivetrain.mapleSimDrive,
					Inches.of(22),
					IntakeSimulation.IntakeSide.FRONT,
					50);
		}
		intakeSimulation.startIntake();
	}

	@Override
	public void periodic() {
		super.periodic();
		if (Robot.isSimulation() && intakeSimulation != null) {
			if (nearPosition(IntakeConstants.converter.toAngle(IntakeConstants.kDeployPosition))) {
				//intakeSimulation.startIntake();
			} else {
				//intakeSimulation.stopIntake();
			}
			SmartDashboard.putNumber("Sim/IntakeCount", intakeSimulation.getGamePiecesAmount());
		}
	}

	public boolean simHasPiece() {
		if (Robot.isSimulation() && intakeSimulation != null) {
			return intakeSimulation.getGamePiecesAmount() > 0;
		}
		return false;
	}

	public void simTakePiece() {
		if (Robot.isSimulation() && intakeSimulation != null) {
			intakeSimulation.obtainGamePieceFromIntake();
		}
	}

	@Override
	public void outputTelemetry() {
		super.outputTelemetry();
		intakePublisher.set(new Pose3d().plus(new Transform3d(
				new Translation3d(
						0.0,
						0.0,
						IntakeConstants.converter.toDistance(getPosition()).in(Units.Meters)),
				new Rotation3d())));
	}
}
