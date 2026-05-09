package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.FollowYLineSwerveRequestCommand;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.util.Util;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import java.util.Set;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.launcher.LauncherConstants;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;

public class Superstructure extends SubsystemBase {
	public static final Superstructure mInstance = new Superstructure();

	private static final double TRENCH_BACKUP_DISTANCE = 0.4;

	private static final Translation2d[] TRENCH_CENTERS = {
			FieldConstants.RightTrench,
			FieldConstants.LeftTrench,
			AllianceFlipUtil.forceApply(FieldConstants.LeftTrench),
			AllianceFlipUtil.forceApply(FieldConstants.RightTrench)
	};

	public static final Translation2d[] ALLIANCE_TRENCH_CENTERS = {
			FieldConstants.RightTrench,
			FieldConstants.LeftTrench
	};

	private boolean driveReady = false;
	private boolean superstructureDone = false;

	private boolean isPathFollowing = false;

	public boolean readyToRaiseElevator = false;

	public Supplier<Rotation2d> hubSupplier = () -> LaunchCalculator.getInstance().getParameters().driveAngle()
			.plus(Rotation2d.k180deg);

	public double lastShootTime = 0;

	private static final double SHOOT_INTERVAL = 0.0909090909;

	@Override
	public void periodic() {
		LaunchCalculator.getInstance().clearLaunchingParameters();
	}

	public void updateTargetedBranch() {
		SwerveDriveState currentState = Drive.mInstance.getState();
		Transform2d speedsPose = new Transform2d(
						currentState.Speeds.vxMetersPerSecond,
						currentState.Speeds.vyMetersPerSecond,
						Rotation2d.fromRadians(currentState.Speeds.omegaRadiansPerSecond))
				.times(SuperstructureConstants.lookaheadBranchSelectionTime.in(Units.Seconds));
		Pose2d lookeaheadPose = currentState.Pose.transformBy(speedsPose);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);

		builder.addDoubleProperty("Battery Voltage", () -> RobotController.getBatteryVoltage(), null);
	}

	public void setDriveReady(boolean valToSet) {
		driveReady = valToSet;
	}

	public void setSuperstructureDone(boolean valToSet) {
		superstructureDone = valToSet;
	}

	public boolean getDriveReady() {
		return driveReady;
	}

	public boolean getSuperstructureDone() {
		return superstructureDone;
	}

	public void setPathFollowing(boolean isFollowing) {
		isPathFollowing = isFollowing;
	}

	public Command autoPassTrench() {
		return autoPassTrench(TRENCH_CENTERS);
	}

	public Command autoPassTrench(Translation2d[] trenchCenters) {
		return new DeferredCommand(
				() -> {
					Pose2d pose = Drive.mInstance.getPose();

					// Find the closest trench center
					Translation2d closestCenter = trenchCenters[0];
					double minDistanceSq = Util.getDistanceSq(pose.getTranslation(), closestCenter);
					for (int i = 1; i < trenchCenters.length; i++) {
						double distanceSq = Util.getDistanceSq(pose.getTranslation(), trenchCenters[i]);
						if (distanceSq < minDistanceSq) {
							minDistanceSq = distanceSq;
							closestCenter = trenchCenters[i];
						}
					}

					double targetY = closestCenter.getY();
					double x_c = closestCenter.getX();

					double gateStartX = x_c - 1.2;
					double gateEndX = x_c + 1.2;

					double distToStart = Math.abs(pose.getX() - gateStartX);
					double distToEnd = Math.abs(pose.getX() - gateEndX);
					boolean reversed = distToEnd < distToStart;

					double targetX;
					double velocityX;

					if (reversed) {
						targetX = gateEndX;
						velocityX = -5.0;
					} else {
						targetX = gateStartX;
						velocityX = 5.0;
					}

					if (AllianceFlipUtil.shouldFlip()) {
						velocityX = -velocityX;
					}

					Rotation2d targetRotation = frc.robot.autos.AutoHelpers.snapRotationTrench(pose.getRotation());
					Command traversal = new FollowYLineSwerveRequestCommand(targetY, velocityX, targetRotation)
							.withTimeout(1.0);

					boolean insideX = pose.getX() > gateStartX && pose.getX() < gateEndX;
					boolean outsideY = Math.abs(pose.getY() - targetY)
							> FieldConstants.RightTrenchDimensions.openingWidth / 2.0;

					if (insideX && outsideY) {
						double backupX = reversed
								? gateEndX + TRENCH_BACKUP_DISTANCE
								: gateStartX - TRENCH_BACKUP_DISTANCE;
						Pose2d backupPose = new Pose2d(backupX, targetY, targetRotation);
						return new PIDToPoseCommand(backupPose, Centimeters.of(25.0), Degrees.of(15),
								DriveConstants.mAutoAlignTranslationController,
								DriveConstants.mAutoAlignHeadingController).andThen(traversal);
					}

					if (insideX) {
						return traversal;
					}

					Pose2d approachPose = new Pose2d(targetX, targetY, targetRotation);
					return new PIDToPoseCommand(approachPose, Centimeters.of(25.0), Degrees.of(15),
							DriveConstants.mAutoAlignTranslationController,
							DriveConstants.mAutoAlignHeadingController).andThen(traversal);
				},
				Set.of(Drive.mInstance));
	}

	public void SimCalcShoot() {
		double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		if (lastShootTime == 0) {
			lastShootTime = currentTime;
		}
		double elapsedTime = currentTime - lastShootTime;
		int ballsToShoot = (int) (elapsedTime / SHOOT_INTERVAL);
		for (int i = 0; i < Math.min(ballsToShoot, 5); i++) {
			SimShoot();
		}
		if (ballsToShoot > 0) {
			lastShootTime += ballsToShoot * SHOOT_INTERVAL;
		}
	}

	public void SimShoot() {
		if (!Intake.mInstance.simHasPiece()) {
			return;
		}
		Intake.mInstance.simTakePiece();

		LaunchCalculator.getInstance().clearLaunchingParameters();
		var params = LaunchCalculator.getInstance().getParameters();

		double flywheelRadsPerSec = params.flywheelSpeed()*1.45;
		double hoodAngleRads = params.hoodAngle();

		RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
				Drive.mInstance.getPose().getTranslation(),
				GeomUtil.toTransform2d(LauncherConstants.robotToLauncher).getTranslation()
						.plus(new Translation2d(
								(Math.random() * 0.5) - 0.25,
								(Math.random() * 0.5) - 0.25)),
				Drive.mInstance.getFieldRelativeSpeeds(),
				Drive.mInstance.getHeading(),
				Meters.of(0.55),
				MetersPerSecond.ofBaseUnits(flywheelRadsPerSec * 0.0325 * 1.17 * 1.05 * 0.75),
				Radians.of(Math.PI / 2).minus(Radians.of(hoodAngleRads)));
		fuelOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();
		SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
	}

}
