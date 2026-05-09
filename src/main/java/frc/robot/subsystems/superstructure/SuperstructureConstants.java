package frc.robot.subsystems.superstructure;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.io.BeamBreakIO;
import frc.lib.io.BeamBreakIODigitalIn;
import frc.lib.io.BeamBreakIOSim;
import frc.lib.util.FieldLayout.Level;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.controlboard.ControlBoardConstants;
import java.util.function.BooleanSupplier;

public final class SuperstructureConstants {
	public static class BeamBreakConstants {
		public static BeamBreakIO getEndEffectorCoralBeamBreak() {
			if (Robot.isReal()) {
				try {
					return new BeamBreakIODigitalIn(
							Ports.END_EFFECTOR_CORAL_BREAMBREAK.id,
							SuperstructureConstants.kEndEffectorCoralDebounce,
							"Coral End Effector Break");
				} catch (Exception e) {
					SmartDashboard.putString("End Effector Beam Break", "Failed");
					return new BeamBreakIOSim(
							() -> false, SuperstructureConstants.kEndEffectorCoralDebounce, "Coral End Effector Break");
				}
			} else {
				return new BeamBreakIOSim(
						ControlBoardConstants.mOperatorController.povDownRight().or(() -> Robot.isSimulation()),
						SuperstructureConstants.kEndEffectorCoralDebounce,
						"Coral End Effector Break");
			}
		}

		public static BeamBreakIO getEndEffectorAlgaeBeamBreak() {
			if (Robot.isReal()) {
				try {
					return new BeamBreakIODigitalIn(
							Ports.END_EFFECTOR_ALGAE_BEAMBREAK.id,
							SuperstructureConstants.kEndEffectorAlgaeDebounce,
							"Algae End Effector Break");
				} catch (Exception e) {
					SmartDashboard.putString("End Effector Beam Break", "Failed");
					return new BeamBreakIOSim(
							() -> false, SuperstructureConstants.kEndEffectorAlgaeDebounce, "Algae End Effector Break");
				}
			} else {
				return new BeamBreakIOSim(
						ControlBoardConstants.mOperatorController.povUpLeft(),
						SuperstructureConstants.kEndEffectorAlgaeDebounce,
						"Algae End Effector Break");
			}
		}

	}

	public static final Angle lookingAwayFromReefAfterL1Threshold = Units.Degrees.of(60.0);
	public static final Distance farFromReefAfterL1Threshold = Units.Inches.of(76.0);

	/* Auto Align Tuning Values */
	public static final Time lookaheadBranchSelectionTime = Units.Milliseconds.of(100.0);

	public static final Distance kElevatorCenterOffset = Units.Inches.of(12.5);

	public static final Distance kAlgaeOffsetFactor = Units.Centimeters.of(10.0);
	public static final Distance kAlgaeReadyOffsetFactor = Units.Centimeters.of(20.0);
	public static final Distance kL4CoralOffsetFactor = Units.Centimeters.of(34.25);
	public static final Distance kL3CoralOffsetFactor = Units.Centimeters.of(30.25);
	public static final Distance kL2CoralOffsetFactor = Units.Centimeters.of(30.25);
	public static final Distance kL1CoralOffsetFactor = Units.Centimeters.of(60.0);

	public static final Distance kL1CoralHorizontalOffsetFactor = Units.Inches.of(6.469);

	public static final Time kClimberDebounce = Units.Seconds.of(0.04);
	public static final Time kEndEffectorCoralDebounce = Units.Seconds.of(0.04);
	public static final Time kIndexerDebounce = Units.Seconds.of(0.04);
	public static final Time kCoralRollersCurrentSpikeDebounce = Units.Seconds.of(0.4);
	public static final Time kEndEffectorAlgaeDebounce = Units.Seconds.of(0.09);
	public static final Time kCoralRollersVelocityDebounce = Units.Seconds.of(0.04);

	public static final AngularVelocity kPivotStableThresholdVelocity = Units.DegreesPerSecond.of(20.0);

	public static final AngularVelocity kEndEffectorVelocityDip = Units.DegreesPerSecond.of(2000);

	public static final Distance kAlgaeStowReefDistance = Units.Meters.of(2.0);

	public static final Time kRecentUpdateTime = Units.Seconds.of(0.1);
	public static final Distance kNearUpdateDistance = Units.Centimeters.of(2.0);

}
