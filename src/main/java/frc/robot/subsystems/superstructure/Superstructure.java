package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.drive.FollowXLineSwerveRequestCommand;
import frc.lib.drive.PIDToPoseCommand;
import frc.lib.io.BeamBreakIO;
import frc.lib.io.BeamBreakIOSim;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Branch.Face;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.Util;
import frc.robot.RobotConstants;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants.BeamBreakConstants;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.launcher.LauncherConstants;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;

public class Superstructure extends SubsystemBase {
	public static final Superstructure mInstance = new Superstructure();

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

		double flywheelRadsPerSec = params.flywheelSpeed();
		double hoodAngleRads = params.hoodAngle();

		RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
				Drive.mInstance.getPose().getTranslation(),
				GeomUtil.toTransform2d(LauncherConstants.robotToLauncher).getTranslation()
						.plus(new Translation2d(
								(Math.random() * 0.5) - 0.25,
								(Math.random() * 0.5) - 0.25)),
				Drive.mInstance.getFieldRelativeSpeeds(),
				Drive.mInstance.getHeading().plus(Rotation2d.k180deg),
				Meters.of(0.55),
				MetersPerSecond.ofBaseUnits(flywheelRadsPerSec * 0.0325 * 1.17 * 1.05 * 0.75),
				Radians.of(Math.PI / 2).minus(Radians.of(hoodAngleRads)));
		fuelOnFly.enableBecomesGamePieceOnFieldAfterTouchGround();
		SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
	}

}
