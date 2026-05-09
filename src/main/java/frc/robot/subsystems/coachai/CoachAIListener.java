package frc.robot.subsystems.coachai;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.PIDToPoseCommand;
import frc.robot.commands.AutoIntakeFuelCommand;
import frc.robot.commands.AutoalignThenShootCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.ArrayList;
import java.util.Random;
import java.util.Set;
import org.littletonrobotics.frc2026.FieldConstants;

public class CoachAIListener extends SubsystemBase {
	public static final CoachAIListener mInstance = new CoachAIListener();

	private final NetworkTable table;

	private final StringSubscriber actionSub;
	private final StringSubscriber actionArgSub;
	private final StringSubscriber modeSub;
	private final IntegerSubscriber actionSeqSub;

	private final BooleanSubscriber zoneAllianceLeftSub;
	private final BooleanSubscriber zoneAllianceRightSub;
	private final BooleanSubscriber zoneMiddleLeftSub;
	private final BooleanSubscriber zoneMiddleRightSub;
	private final BooleanSubscriber zoneOpponentLeftSub;
	private final BooleanSubscriber zoneOpponentRightSub;

	private static final Random random = new Random();

	private long lastProcessedSeq = 0;
	private Command currentAICommand = null;
	private String currentMode = "intaking";
	private String previousMode = "intaking";

	private CoachAIListener() {
		table = NetworkTableInstance.getDefault().getTable("CoachAI");

		actionSub = table.getStringTopic("action").subscribe("idle");
		actionArgSub = table.getStringTopic("action_arg").subscribe("");
		modeSub = table.getStringTopic("mode").subscribe("intaking");
		actionSeqSub = table.getIntegerTopic("action_seq").subscribe(0);

		zoneAllianceLeftSub = table.getBooleanTopic("zone/alliance_left").subscribe(true);
		zoneAllianceRightSub = table.getBooleanTopic("zone/alliance_right").subscribe(true);
		zoneMiddleLeftSub = table.getBooleanTopic("zone/middle_left").subscribe(true);
		zoneMiddleRightSub = table.getBooleanTopic("zone/middle_right").subscribe(true);
		zoneOpponentLeftSub = table.getBooleanTopic("zone/opponent_left").subscribe(true);
		zoneOpponentRightSub = table.getBooleanTopic("zone/opponent_right").subscribe(true);
	}

	@Override
	public void periodic() {
		// Update zone enables from NT
		FieldConstants.ALLIANCE_LEFT.setEnabled(zoneAllianceLeftSub.get());
		FieldConstants.ALLIANCE_RIGHT.setEnabled(zoneAllianceRightSub.get());
		FieldConstants.MIDDLE_LEFT.setEnabled(zoneMiddleLeftSub.get());
		FieldConstants.MIDDLE_RIGHT.setEnabled(zoneMiddleRightSub.get());
		FieldConstants.OPPONENT_LEFT.setEnabled(zoneOpponentLeftSub.get());
		FieldConstants.OPPONENT_RIGHT.setEnabled(zoneOpponentRightSub.get());

		// Update mode - restart command if mode changed
		currentMode = modeSub.get();
		if (!currentMode.equals(previousMode)) {
			previousMode = currentMode;
			applyMode(currentMode);
		}

		// Check for new action
		long seq = actionSeqSub.get();
		if (seq > lastProcessedSeq) {
			lastProcessedSeq = seq;
			String action = actionSub.get();
			String arg = actionArgSub.get();
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
				currentAICommand = new AutoIntakeFuelCommand();
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
				currentAICommand = new AutoIntakeFuelCommand();
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
	 * If the robot is not in an alliance zone, autotrench back first, then shoot.
	 * If only alliance_left is active, trench to the left side.
	 * If only alliance_right is active, trench to the right side.
	 */
	private Command buildShootCommand() {
		Superstructure s = Superstructure.mInstance;
		FieldConstants.Zone currentZone = FieldConstants.getZone(Drive.mInstance.getPose());

		boolean inAllianceZone = currentZone == FieldConstants.ALLIANCE_LEFT
				|| currentZone == FieldConstants.ALLIANCE_RIGHT;

		if (inAllianceZone) {
			return new AutoalignThenShootCommand(s.hubSupplier);
		}

		// Not in alliance zone: autotrench back to alliance, then shoot
		return s.autoPassTrench()
				.andThen(new AutoalignThenShootCommand(s.hubSupplier));
	}

	/**
	 * Pass through trench. If only alliance_left is enabled, go left after trench.
	 * If only alliance_right is enabled, go right after trench.
	 */
	private Command buildPassCommand() {
		return Superstructure.mInstance.autoPassTrench();
	}

	/**
	 * Drive to a random point in an enabled zone, then pick another one and repeat.
	 */
	private Command buildDefendCommand() {
		return Commands.sequence(
				Commands.defer(() -> {
					Pose2d target = getRandomEnabledZonePose();
					if (target == null) {
						return Commands.none();
					}
					return new PIDToPoseCommand(
							target,
							Centimeters.of(30),
							Degrees.of(20),
							DriveConstants.mAutoAlignTranslationController,
							DriveConstants.mAutoAlignHeadingController);
				}, Set.of(Drive.mInstance))
		).repeatedly();
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
