package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.drive.DetectionPIDToPoseCommand;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.Stopwatch;
import frc.robot.RobotConstants;
import frc.robot.autos.AutoConstants.AutoType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

public class AutoModeBase {
	private static AutoRoutine routine;
	private static Stopwatch stopwatch = new Stopwatch();
	private static AutoType side;

	public AutoModeBase(AutoFactory factory, String name) {
		routine = factory.newRoutine(name);
	}

	public AutoModeBase(AutoFactory factory, String name, AutoType side) {
		this(factory, name);
		AutoModeBase.side = side;
	}

	/**
	 * @return Trajectory from choreo
	 */
	public AutoTrajectory trajectory(String name) {
		return routine.trajectory(name);
	}

	public AutoTrajectory trajectory(String name, int index) {
		return routine.trajectory(name, index);
	}

	/**
	 * Runs an accuracy-based command for choreo following
	 *
	 * @param trajectory
	 * @param timeout
	 */
	public static Command cmdWithRotationAccuracy(AutoTrajectory trajectory, Time timeout) {
		return Commands.defer(
						() -> new FunctionalCommand(
								trajectory.cmd()::initialize,
								trajectory.cmd()::execute,
								trajectory.cmd()::end,
								() -> rotationIsFinished(trajectory)),
						Set.of(Drive.mInstance))
				.beforeStarting(() -> Superstructure.mInstance.setDriveReady(false))
				.withTimeout(trajectory.getRawTrajectory().getTotalTime() + timeout.in(Units.Seconds));
	}

	public static Command cmdWithRotationAccuracy(AutoTrajectory trajectory) {
		return cmdWithRotationAccuracy(trajectory, AutoConstants.kDefaultTrajectoryTimeout);
	}

	/**
	 * Runs an accuracy-based command for choreo following
	 *
	 * @param trajectory
	 * @param timeout
	 */
	public static Command cmdWithAccuracy(AutoTrajectory trajectory, Time timeout, Distance epsilonDist) {
		return Commands.defer(
						() -> new FunctionalCommand(
								trajectory.cmd()::initialize,
								trajectory.cmd()::execute,
								trajectory.cmd()::end,
								() -> isFinished(trajectory, epsilonDist)),
						Set.of(Drive.mInstance))
				.beforeStarting(() -> Superstructure.mInstance.setDriveReady(false))
				.withTimeout(trajectory.getRawTrajectory().getTotalTime() + timeout.in(Units.Seconds));
	}

	/**
	 * Returns an accuracy-based command for choreo following, including the default timeout
	 *
	 * @param trajectory
	 */
	public static Command cmdWithAccuracy(AutoTrajectory trajectory, Distance epsilonDist) {
		return cmdWithAccuracy(trajectory, AutoConstants.kDefaultTrajectoryTimeout, epsilonDist);
	}

	public static Command cmdWithAccuracy(AutoTrajectory trajectory) {
		return cmdWithAccuracy(trajectory, AutoConstants.kAutoLinearEpsilon);
	}

	public static Command cmdWithInterrupt(AutoTrajectory trajectory) {
		return trajectory.cmd().handleInterrupt(() -> {
			Drive.mInstance.setSwerveRequest(new SwerveRequest.ApplyRobotSpeeds());
			SmartDashboard.putNumber("Auto Trajectory Command Interrupted", Timer.getFPGATimestamp());
		});
	}

	private static boolean rotationIsFinished(AutoTrajectory trajectory) {
		Pose2d currentPose = Drive.mInstance.getPose();
		Pose2d finalPose = trajectory.getFinalPose().get();
		Angle epsilonAngle = AutoConstants.kAutoAngleEpsilon;

		return MathUtil.angleModulus(Math.abs(
						currentPose.getRotation().minus(finalPose.getRotation()).getRadians()))
				< epsilonAngle.in(Units.Radians);
	}

	private static boolean translationIsFinished(AutoTrajectory trajectory, Distance epsilonDist) {
		Pose2d currentPose = Drive.mInstance.getPose();
		Pose2d finalPose = trajectory.getFinalPose().get();

		SmartDashboard.putNumber(
				"Choreo/Distance Away Inches",
				currentPose.getTranslation().getDistance(finalPose.getTranslation()) * 39.37);

		return currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist.in(Units.Meters);
	}

	private static boolean isFinished(AutoTrajectory trajectory, Distance epsilonDist) {
		boolean translationCompleted = translationIsFinished(trajectory, epsilonDist);
		boolean rotationCompleted = rotationIsFinished(trajectory);

		SmartDashboard.putBoolean("Choreo/Translation Completed", translationCompleted);
		SmartDashboard.putBoolean("Choreo/Rotation Completed", rotationCompleted);

		if (translationCompleted && rotationCompleted) {
			stopwatch.startIfNotRunning();
			if (stopwatch.getTime().gte(AutoConstants.kDelayTime)) {
				stopwatch.reset();
				return true;
			}
		} else if (!translationCompleted || !rotationCompleted) {
			stopwatch.reset();
		}

		SmartDashboard.putNumber("Choreo/Stopwatch Time", stopwatch.getTimeAsDouble());
		return false;
	}
	
	public void prepRoutine(Command... sequence) {
		routine.active()
				.onTrue(Commands.sequence(sequence)
						.alongWith(Commands.runOnce(() -> Elevator.mInstance.setCurrentPosition(
								ElevatorConstants.converter.toAngle(ElevatorConstants.kStowPosition))))
						.withName("Auto Routine Sequential Command Group"));
	}

	public void atTranslation(String eventName, Command event, Distance epsilon, AutoTrajectory... trajectories) {
		for (AutoTrajectory trajectory : trajectories) {
			trajectory.atTranslation(eventName, epsilon.in(Units.Meters)).onTrue(event);
		}
	}

	public void atTranslation(String eventName, Command event, AutoTrajectory... trajectories) {
		atTranslation(eventName, event, AutoConstants.kAutoLinearEpsilon, trajectories);
	}

	public void logTrajectories(AutoTrajectory... trajectories) {
		List<AutoTrajectory> list = Arrays.asList(trajectories);
		for (int i = 1; i <= list.size(); ++i) {
			if (RobotConstants.isRedAlliance) {
				LogUtil.recordTrajectory(
						"Autos/Choreo Path " + i,
						list.get(i - 1).getRawTrajectory().flipped());
			} else {
				LogUtil.recordTrajectory(
						"Autos/Choreo Path " + i, list.get(i - 1).getRawTrajectory());
			}
		}
	}

	public AutoRoutine getRoutine() {
		return routine;
	}

	public Command asCommand() {
		return routine.cmd();
	}
}
