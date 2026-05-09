package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.ArrayList;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.Mock.Logger;

/**
 * While held, PID auto-aligns the robot to face and drive toward the nearest fuel
 * within the robot's current zone. On initialize, all zones are disabled except the
 * one the robot is currently in. On end, all zones are re-enabled.
 */
public class AutoIntakeFuelCommand extends Command {

	public AutoIntakeFuelCommand() {
		addRequirements(Drive.mInstance);
	}

	@Override
	public void initialize() {
		FieldConstants.enableOnlyCurrentZone(Drive.mInstance.getPose());
	}

	@Override
	public void execute() {
		Translation2d nearest = getNearestFuelInZone();

		Logger.recordOutput("AutoIntake/HasTarget", nearest != null);
		if (nearest != null) {
			Logger.recordOutput("AutoIntake/TargetPose", new Pose2d(nearest, Rotation2d.kZero));
		}

		if (nearest == null) {
			Drive.mInstance.setSwerveRequest(new SwerveRequest.SwerveDriveBrake());
			return;
		}

		Translation2d robotPos = Drive.mInstance.getPose().getTranslation();
		Rotation2d facingAngle = nearest.minus(robotPos).getAngle();
		Pose2d drivePose = new Pose2d(nearest, facingAngle);

		Drive.mInstance.setSwerveRequest(
				DriveConstants.getPIDToPoseRequestUpdater(drivePose)
						.apply(DriveConstants.PIDToPoseRequest));
	}

	private Translation2d getNearestFuelInZone() {
		if (Robot.isSimulation()) {
			return getNearestFuelSim();
		}
		if (!Detection.mInstance.hasCoral()) return null;
		Translation2d fuelPos = Detection.mInstance.getCoralTranslationAndPoint().getTranslation();
		if (!FieldConstants.isInEnabledZone(new Pose2d(fuelPos, Rotation2d.kZero))) return null;
		return fuelPos;
	}

	private Translation2d getNearestFuelSim() {
		Pose3d[] fuels = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
		if (fuels == null || fuels.length == 0) {
			Logger.recordOutput("AutoIntake/ZoneFuels", new Pose3d[0]);
			return null;
		}

		Translation2d robotPos = Drive.mInstance.getPose().getTranslation();
		Translation2d closest = null;
		double closestDist = Double.MAX_VALUE;
		ArrayList<Pose3d> zoneFuels = new ArrayList<>();

		for (Pose3d fuel : fuels) {
			Translation2d fuelPos = fuel.toPose2d().getTranslation();
			if (!FieldConstants.isInEnabledZone(fuelPos)) continue;
			zoneFuels.add(fuel);
			double dist = fuelPos.getDistance(robotPos);
			if (dist < closestDist) {
				closestDist = dist;
				closest = fuelPos;
			}
		}

		Logger.recordOutput("AutoIntake/ZoneFuels", zoneFuels.toArray(new Pose3d[0]));
		return closest;
	}

	@Override
	public void end(boolean interrupted) {
		FieldConstants.enableAllZones();
		Drive.mInstance.setSwerveRequest(new SwerveRequest.ApplyRobotSpeeds());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
