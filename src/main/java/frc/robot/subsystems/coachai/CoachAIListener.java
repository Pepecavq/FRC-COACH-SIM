package frc.robot.subsystems.coachai;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.PIDToPoseCommand;
import frc.robot.Robot;
import frc.robot.commands.AutoalignThenShootCommand;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.ArrayList;
import java.util.Random;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.frc2026.FieldConstants;

public class CoachAIListener extends SubsystemBase {
	public static final CoachAIListener mInstance = new CoachAIListener();

	private final NetworkTableEntry actionEntry;
	private final NetworkTableEntry actionArgEntry;
	private final NetworkTableEntry modeEntry;
	private final NetworkTableEntry actionSeqEntry;

	private final NetworkTableEntry zoneAllianceLeftEntry;
	private final NetworkTableEntry zoneAllianceRightEntry;
	private final NetworkTableEntry zoneMiddleLeftEntry;
	private final NetworkTableEntry zoneMiddleRightEntry;
	private final NetworkTableEntry zoneOpponentLeftEntry;
	private final NetworkTableEntry zoneOpponentRightEntry;

	private static final Random random = new Random();

	private long lastProcessedSeq = 0;
	private Command currentAICommand = null;
	private String currentMode = "intaking";
	private String previousMode = "";

	private CoachAIListener() {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("CoachAI");

		actionEntry = table.getEntry("action");
		actionArgEntry = table.getEntry("action_arg");
		modeEntry = table.getEntry("mode");
		actionSeqEntry = table.getEntry("action_seq");

		zoneAllianceLeftEntry = table.getEntry("zone/alliance_left");
		zoneAllianceRightEntry = table.getEntry("zone/alliance_right");
		zoneMiddleLeftEntry = table.getEntry("zone/middle_left");
		zoneMiddleRightEntry = table.getEntry("zone/middle_right");
		zoneOpponentLeftEntry = table.getEntry("zone/opponent_left");
		zoneOpponentRightEntry = table.getEntry("zone/opponent_right");
	}

	@Override
	public void periodic() {
		// Update zone enables from NT
		FieldConstants.ALLIANCE_LEFT.setEnabled(zoneAllianceLeftEntry.getBoolean(true));
		FieldConstants.ALLIANCE_RIGHT.setEnabled(zoneAllianceRightEntry.getBoolean(true));
		FieldConstants.MIDDLE_LEFT.setEnabled(zoneMiddleLeftEntry.getBoolean(true));
		FieldConstants.MIDDLE_RIGHT.setEnabled(zoneMiddleRightEntry.getBoolean(true));
		FieldConstants.OPPONENT_LEFT.setEnabled(zoneOpponentLeftEntry.getBoolean(true));
		FieldConstants.OPPONENT_RIGHT.setEnabled(zoneOpponentRightEntry.getBoolean(true));

		// Update mode - restart command if mode changed
		currentMode = modeEntry.getString("intaking");
		SmartDashboard.putString("CoachAI/currentMode", currentMode);
		SmartDashboard.putString("CoachAI/previousMode", previousMode);
		SmartDashboard.putBoolean("CoachAI/hasCommand", currentAICommand != null);
		SmartDashboard.putBoolean("CoachAI/commandScheduled",
				currentAICommand != null && currentAICommand.isScheduled());

		if (!currentMode.equals(previousMode)) {
			SmartDashboard.putString("CoachAI/lastModeSwitch", currentMode);
			previousMode = currentMode;
			applyMode(currentMode);
		}

		// Check for new action
		long seq = (long) actionSeqEntry.getInteger(0);
		SmartDashboard.putNumber("CoachAI/actionSeq", seq);
		if (seq > lastProcessedSeq) {
			lastProcessedSeq = seq;
			String action = actionEntry.getString("idle");
			String arg = actionArgEntry.getString("");
			handleAction(action, arg);
		}
	}

	private void applyMode(String mode) {
		cancelCurrentCommand();

		switch (mode) {
			case "shooting":
				currentAICommand = buildShootCommand();
				currentAICommand.schedule();
				break;
			case "passing":
				currentAICommand = buildPassCommand();
				currentAICommand.schedule();
				break;
			case "intaking":
				currentAICommand = buildIntakeCommand();
				currentAICommand.schedule();
				break;
			case "defend":
				currentAICommand = buildDefendCommand();
				currentAICommand.schedule();
				break;
			default:
				break;
		}
	}

	private void handleAction(String action, String arg) {
		switch (action) {
			case "intake":
				cancelCurrentCommand();
				currentAICommand = buildIntakeCommand();
				currentAICommand.schedule();
				break;

			case "shoot":
				if ("true".equalsIgnoreCase(arg)) {
					cancelCurrentCommand();
					currentAICommand = buildShootCommand();
					currentAICommand.schedule();
				}
				break;

			case "pass":
				if ("true".equalsIgnoreCase(arg)) {
					cancelCurrentCommand();
					currentAICommand = buildPassCommand();
					currentAICommand.schedule();
				}
				break;

			case "defend":
				cancelCurrentCommand();
				currentAICommand = buildDefendCommand();
				currentAICommand.schedule();
				break;

			default:
				break;
		}
	}

	/**
	 * Find nearest fuel on the field. If it's in a different region than the robot,
	 * trench through boundaries to get there first, then PID to it.
	 * Repeats so the robot keeps seeking fuel across zones.
	 */
	private Command buildIntakeCommand() {
		return Commands.sequence(
				Commands.defer(() -> {
					Pose2d robotPose = Drive.mInstance.getPose();
					int robotRegion = getRegion(robotPose);

					Translation2d nearestFuel = getNearestFuelAnywhere();

					SmartDashboard.putBoolean("CoachAI/intake/hasFuel", nearestFuel != null);
					SmartDashboard.putNumber("CoachAI/intake/robotRegion", robotRegion);
					SmartDashboard.putNumber("CoachAI/intake/allianceTrenchX", FieldConstants.allianceTrenchX);
					SmartDashboard.putNumber("CoachAI/intake/opponentTrenchX", FieldConstants.opponentTrenchX);
					SmartDashboard.putNumber("CoachAI/intake/robotX", robotPose.getX());

					if (nearestFuel == null) {
						return Commands.waitSeconds(0.25);
					}

					int fuelRegion = getRegion(nearestFuel.getX());

					SmartDashboard.putNumber("CoachAI/intake/fuelX", nearestFuel.getX());
					SmartDashboard.putNumber("CoachAI/intake/fuelY", nearestFuel.getY());
					SmartDashboard.putNumber("CoachAI/intake/fuelRegion", fuelRegion);

					Rotation2d facingAngle = nearestFuel.minus(
							robotPose.getTranslation()).getAngle();
					Pose2d fuelPose = new Pose2d(nearestFuel, facingAngle);

					if (fuelRegion == robotRegion) {
						SmartDashboard.putString("CoachAI/intake/action", "PID direct");
						return new PIDToPoseCommand(
								fuelPose,
								Centimeters.of(15),
								Degrees.of(15),
								DriveConstants.mAutoAlignTranslationController,
								DriveConstants.mAutoAlignHeadingController);
					}

					SmartDashboard.putString("CoachAI/intake/action", "trench then drive");
					return buildTrenchThenDrive(fuelPose);
				}, Set.of(Drive.mInstance))
		).repeatedly();
	}

	/**
	 * Find the nearest fuel on the entire field regardless of zone.
	 */
	private Translation2d getNearestFuelAnywhere() {
		if (!Robot.isSimulation()) {
			if (!Detection.mInstance.hasCoral()) return null;
			return Detection.mInstance.getCoralTranslationAndPoint().getTranslation();
		}

		Pose3d[] fuels = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
		if (fuels == null || fuels.length == 0) {
			SmartDashboard.putNumber("CoachAI/intake/fuelCount", 0);
			return null;
		}

		SmartDashboard.putNumber("CoachAI/intake/fuelCount", fuels.length);

		Translation2d robotPos = Drive.mInstance.getPose().getTranslation();
		Translation2d closest = null;
		double closestDist = Double.MAX_VALUE;

		// Log all fuel positions as Pose2d array for field display
		Pose2d[] fuelPoses = new Pose2d[fuels.length];
		for (int i = 0; i < fuels.length; i++) {
			Translation2d fuelPos = fuels[i].toPose2d().getTranslation();
			fuelPoses[i] = new Pose2d(fuelPos, Rotation2d.kZero);
			if (fuelPos.getX() < 0 || fuelPos.getY() < 0) continue;
			double dist = fuelPos.getDistance(robotPos);
			if (dist < closestDist) {
				closestDist = dist;
				closest = fuelPos;
			}
		}

		SmartDashboard.putData("CoachAI/intake/allFuelField",
				builder -> builder.addDoubleArrayProperty("poses",
						() -> posesToArray(fuelPoses), null));

		if (closest != null) {
			SmartDashboard.putString("CoachAI/intake/targetFuel",
					String.format("(%.2f, %.2f)", closest.getX(), closest.getY()));
		}

		return closest;
	}

	private double[] posesToArray(Pose2d[] poses) {
		double[] arr = new double[poses.length * 3];
		for (int i = 0; i < poses.length; i++) {
			arr[i * 3] = poses[i].getX();
			arr[i * 3 + 1] = poses[i].getY();
			arr[i * 3 + 2] = poses[i].getRotation().getRadians();
		}
		return arr;
	}

	/**
	 * If the robot is not in an alliance zone, trench back to alliance first.
	 * Then shoot continuously (runs until cancelled by a mode change).
	 * AutoalignThenShootCommand relies on the default drive command to auto-aim,
	 * so it must NOT be in a group that holds Drive.
	 */
	private Command buildShootCommand() {
		Superstructure s = Superstructure.mInstance;
		int currentRegion = getRegion(Drive.mInstance.getPose());

		if (currentRegion == 0) {
			return new AutoalignThenShootCommand(s.hubSupplier);
		}

		// Trench back to alliance first, then schedule shoot as a separate command
		// so it doesn't hold Drive (AutoalignThenShootCommand needs the default drive
		// command to run for auto-aim)
		Command trenchCmd = Commands.none();
		for (int r = currentRegion; r > 0; r--) {
			trenchCmd = trenchCmd.andThen(s.autoPassTrench(Superstructure.ALLIANCE_TRENCH_CENTERS).withTimeout(5.0));
		}
		return trenchCmd.andThen(
				Commands.runOnce(() -> {
					currentAICommand = new AutoalignThenShootCommand(s.hubSupplier);
					currentAICommand.schedule();
				}));
	}

	private Command buildPassCommand() {
		return Commands.sequence(
				Commands.defer(() -> {
					return Superstructure.mInstance.autoPassTrench().withTimeout(5.0);
				}, Set.of(Drive.mInstance))
		).repeatedly();
	}

	/**
	 * Drive to a random point in an enabled zone, trenching through boundaries as needed,
	 * then pick another one and repeat.
	 */
	private Command buildDefendCommand() {
		return Commands.sequence(
				Commands.defer(() -> {
					Pose2d target = getRandomEnabledZonePose();
					if (target == null) {
						return Commands.none();
					}
					return buildTrenchThenDrive(target);
				}, Set.of(Drive.mInstance))
		).repeatedly();
	}

	/**
	 * Returns 0 for alliance, 1 for middle, 2 for opponent based on X position.
	 */
	private int getRegion(double x) {
		if (x < FieldConstants.allianceTrenchX) return 0;
		if (x < FieldConstants.opponentTrenchX) return 1;
		return 2;
	}

	private int getRegion(Pose2d pose) {
		return getRegion(pose.getX());
	}

	/**
	 * Builds a command that trenches through each boundary between the robot's
	 * current region and the target region, then PID drives to the final pose.
	 */
	private Command buildTrenchThenDrive(Pose2d target) {
		Superstructure s = Superstructure.mInstance;
		int currentRegion = getRegion(Drive.mInstance.getPose());
		int targetRegion = getRegion(target);

		Command cmd = Commands.none();

		if (currentRegion < targetRegion) {
			// Moving toward opponent side: trench forward through each boundary
			for (int r = currentRegion; r < targetRegion; r++) {
				cmd = cmd.andThen(s.autoPassTrench().withTimeout(5.0));
			}
		} else if (currentRegion > targetRegion) {
			// Moving toward alliance side: trench backward through each boundary
			for (int r = currentRegion; r > targetRegion; r--) {
				cmd = cmd.andThen(s.autoPassTrench().withTimeout(5.0));
			}
		}

		// After trenching to the correct region, drive to the exact target
		cmd = cmd.andThen(new PIDToPoseCommand(
				target,
				Centimeters.of(30),
				Degrees.of(20),
				DriveConstants.mAutoAlignTranslationController,
				DriveConstants.mAutoAlignHeadingController));

		return cmd;
	}

	private Pose2d getRandomEnabledZonePose() {
		ArrayList<FieldConstants.Zone> enabled = new ArrayList<>();
		for (FieldConstants.Zone zone : FieldConstants.ALL_ZONES) {
			if (zone.isEnabled()) {
				enabled.add(zone);
			}
		}
		if (enabled.isEmpty()) return null;

		FieldConstants.Zone zone = enabled.get(random.nextInt(enabled.size()));
		double x = zone.getMinX() + random.nextDouble() * (zone.getMaxX() - zone.getMinX());
		double y = zone.getMinY() + random.nextDouble() * (zone.getMaxY() - zone.getMinY());
		Rotation2d heading = Rotation2d.fromDegrees(random.nextDouble() * 360.0);
		return new Pose2d(x, y, heading);
	}

	private void cancelCurrentCommand() {
		if (currentAICommand != null && currentAICommand.isScheduled()) {
			currentAICommand.cancel();
		}
		currentAICommand = null;
	}

	public String getCurrentMode() {
		return currentMode;
	}
}
