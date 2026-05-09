package frc.robot.autos;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.lib.drive.TrajectoryHelpers;
import frc.lib.util.FieldLayout;
import frc.lib.util.FieldLayout.Branch;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.Stopwatch;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.Set;

public class AutoHelpers {
	public static Stopwatch stopwatch = new Stopwatch();

	public static void bindEventMarkers(AutoFactory mAutoFactory) {
		Superstructure s = Superstructure.mInstance;
	
	}

	public static String getName(AutoModeSelector mAutoModeSelector) {
		return mAutoModeSelector.getSelectedCommand().getName();
	}

	public static Command resetPoseIfWithoutEstimate(Pose2d pose) {
		return Commands.runOnce(() -> Drive.mInstance.resetPose(pose));
	}

	private static final Rotation2d[] SNAP_ANGLES_TRENCH = {
			Rotation2d.fromDegrees(0),
			Rotation2d.fromDegrees(180)
	};

	public static Rotation2d snapRotationTrench(Rotation2d rotation) {
		Rotation2d bestAngle = SNAP_ANGLES_TRENCH[0];
		double bestDiff = Math.abs(rotation.minus(SNAP_ANGLES_TRENCH[0]).getDegrees());
		for (int i = 1; i < SNAP_ANGLES_TRENCH.length; i++) {
			double diff = Math.abs(rotation.minus(SNAP_ANGLES_TRENCH[i]).getDegrees());
			if (diff < bestDiff) {
				bestDiff = diff;
				bestAngle = SNAP_ANGLES_TRENCH[i];
			}
		}
		return bestAngle;
	}
}
