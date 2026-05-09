package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants.BeamBreakConstants;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
	public static final Superstructure mInstance = new Superstructure();

	private boolean driveReady = false;
	private boolean superstructureDone = false;

	private boolean isPathFollowing = false;

	public boolean readyToRaiseElevator = false;

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

}
