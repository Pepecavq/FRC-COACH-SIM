package frc.lib.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;

public class FollowYLineSwerveRequestCommand extends Command {
	private final double targetY;
	private final double velocityX;
	private final Rotation2d targetRotation;

	public FollowYLineSwerveRequestCommand(double targetY, double velocityX, Rotation2d targetRotation) {
		this.targetY = targetY;
		this.velocityX = velocityX;
		this.targetRotation = targetRotation;
		addRequirements(Drive.mInstance);
	}

	@Override
	public void initialize() {
		Superstructure.mInstance.setDriveReady(false);
		Superstructure.mInstance.setSuperstructureDone(false);
	}

	@Override
	public void execute() {
		Pose2d currentPose = Drive.mInstance.getPose();

		double ySpeed = -DriveConstants.mStayOnLineTranslationController.calculate(targetY - currentPose.getY());

		double rotRate = DriveConstants.mStayOnLineHeadingController.calculate(
				currentPose.getRotation().minus(targetRotation).getRotations());

		Drive.mInstance.setSwerveRequest(DriveConstants.PIDToPoseRequest
				.withVelocityX(velocityX)
				.withVelocityY(ySpeed)
				.withRotationalRate(Units.RotationsPerSecond.of(rotRate)));
	}

	@Override
	public void end(boolean interrupted) {
		Drive.mInstance.setSwerveRequest(new SwerveRequest.FieldCentric());
	}
}
