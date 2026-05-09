package frc.lib.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.logging.LogUtil;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.Util;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryHelpers {
	/**
	 * Uses an arctan to calculate the angle between your robot and your goal translation.
	 *
	 * @param translation The translation component of your target pose.
	 * @return The angle between your robot's position and the goal position. Used for
	 * calculating the initial tangent of a path.
	 */
	public static Rotation2d angleToScore(Translation2d translation) {
		Translation2d scoringTranslation = translation;
		return scoringTranslation
				.minus(Drive.mInstance.getPose().getTranslation())
				.getAngle();
	}

	/**
	 * Uses an arctan to calculate the angle between your robot and your goal translation.
	 *
	 * @param translation The translation component of your target pose.
	 * @return The angle between your robot's position and the goal position. Used for
	 * calculating the initial tangent of a path.
	 */
	public static Rotation2d angleToScoreWithStartPose(Translation2d translation, Pose2d pose) {
		Translation2d scoringTranslation = translation;
		return scoringTranslation.minus(pose.getTranslation()).getAngle();
	}

	/**
	 * Compares your wanted final tangent to where you are on the field, adjusting that
	 * tangent to better work with your path.
	 *
	 * @param translation The translation component of your target pose.
	 * @param idealAngle The ideal approach angle.
	 * @return The final tangent that you should use for your path.
	 */
	public static Rotation2d angleToApproach(Translation2d translation, Rotation2d idealAngle, Angle deadband) {
		Rotation2d currentAngle = angleToScore(translation);
		double currentDegrees = currentAngle.getDegrees();
		double targetDegrees = idealAngle.getDegrees();

		if (idealAngle.getMeasure().isEquivalent(Units.Degrees.of(180))
				&& currentAngle.getMeasure().lte(Units.Degrees.of(0))) currentDegrees += 360.0;

		return Util.epsilonEquals(currentDegrees, targetDegrees, deadband.in(Units.Degrees))
				? Rotation2d.fromDegrees(currentDegrees)
				: idealAngle.plus(Rotation2d.fromDegrees(
						Math.signum(currentDegrees - targetDegrees) * deadband.in(Units.Degrees)));
	}

	/**
	 * Generates a trajectory for your drivetrain to follow based on a target pose.
	 *
	 * @param targetPose The wanted ending pose (x, y, heading).
	 * @return The OTF trajectory to follow based on your current pose and target pose.
	 */
	public static Trajectory generateTrajectoryFromDrive(Pose2d targetPose) {
		List<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(
				new Pose2d(Drive.mInstance.getPose().getTranslation(), angleToScore(targetPose.getTranslation())));
		waypoints.add(targetPose);
		return TrajectoryGenerator.generateTrajectory(waypoints, DriveConstants.getTrajectoryConfig());
	}

	/**
	 * Generates a trajectory for your drivetrain to follow based on a target pose.
	 *
	 * @param targetPose The wanted ending pose (x, y, heading).
	 * @return The OTF trajectory to follow based on your current pose and target pose.
	 */
	public static Trajectory generateTrajectoryFromDriveWithTangent(Pose2d targetPose, Translation2d coralMark) {
		List<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(new Pose2d(Drive.mInstance.getPose().getTranslation(), angleToScore(coralMark)));
		waypoints.add(targetPose);
		return TrajectoryGenerator.generateTrajectory(waypoints, DriveConstants.getTrajectoryConfig());
	}

	/**
	 * Generates a trajectory for your drivetrain to follow based on a target pose.
	 *
	 * @param targetPose The wanted ending pose (x, y, heading).
	 * @return The OTF trajectory to follow based on your current pose and target pose.
	 */
	public static Trajectory generateTrajectory(Pose2d targetPose, Pose2d startPose) {
		List<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(new Pose2d(
				startPose.getTranslation(), angleToScoreWithStartPose(targetPose.getTranslation(), startPose)));
		waypoints.add(targetPose);
		return TrajectoryGenerator.generateTrajectory(waypoints, DriveConstants.getTrajectoryConfig());
	}

	/**
	 * Due to the location this is being applied at, the transform ends up being applied the same way as the opposite method.
	 * This is because these transforms are robot-relative.
	 *
	 * @param wantedFinalPose Wanted final pose of *a gamepiece in the end effector*.
	 * @param offsetLength The distance that the gamepiece is from the center of the robot.
	 * @return The pose the center of your robot will be at.
	 */
	public static Pose2d transformWantedGamepieceToDrivePose(Pose2d wantedFinalPose, Distance offsetLength) {
		Rotation2d r = wantedFinalPose.getRotation();
		Distance x = wantedFinalPose.getMeasureX().minus(offsetLength.times(r.getCos()));
		Distance y = wantedFinalPose.getMeasureY().minus(offsetLength.times(r.getSin()));
		return new Pose2d(x, y, r);
	}

	/**
	 * Due to the location this is being applied at, the transform ends up being applied the same way as the opposite method.
	 * This is because these transforms are robot-relative.
	 *
	 * @param wantedFinalPose Wanted final pose of *the drivetrain*.
	 * @param offsetLength The distance that the center of the robot is from a gamepiece in the end effector.
	 * @return The pose the center of your robot will be at.
	 */
	public static Pose2d transformWantedDriveToGamepiecePose(Pose2d wantedFinalPose, Distance offsetLength) {
		Rotation2d r = wantedFinalPose.getRotation();
		Distance x = wantedFinalPose.getMeasureX().minus(offsetLength.times(r.getCos()));
		Distance y = wantedFinalPose.getMeasureY().minus(offsetLength.times(r.getSin()));
		return new Pose2d(x, y, r);
	}

	public static Trajectory getCoralDetectionTrajectory() {
		if (Detection.mInstance.hasCoral()) {
			Translation2d finalTranslation = Detection.mInstance.getCoralPose().getTranslation();
			Rotation2d finalRotation = finalTranslation
					.minus(Drive.mInstance.getPose().getTranslation())
					.getAngle();
			return generateTrajectory(new Pose2d(finalTranslation, finalRotation), Drive.mInstance.getPose());
		}
		return null;
	}
}
