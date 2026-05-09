package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.bases.ServoMotorSubsystem;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.io.MotorIOTalonFX;

public class Elevator extends ServoMotorSubsystem<MotorIOTalonFX> {
	private final StructPublisher<Pose3d> stage1Publisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Elevator Stage 1", Pose3d.struct)
			.publish();

	private final StructPublisher<Pose3d> stage2Publisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Elevator Stage 2", Pose3d.struct)
			.publish();

	public static final Setpoint IDLE =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(Meters.of(0)));
	public static final Setpoint DEPLOYED =
			Setpoint.withMotionMagicSetpoint(ElevatorConstants.converter.toAngle(Meters.of(0.3)));
	
	public static final Elevator mInstance = new Elevator();

	private Elevator() {
		super(
				ElevatorConstants.getMotorIO(),
				"Elevator",
				ElevatorConstants.converter.toAngle(ElevatorConstants.kEpsilonThreshold),
				ElevatorConstants.getServoConfig());
		setCurrentPosition(ElevatorConstants.converter.toAngle(ElevatorConstants.kStowPosition));
		applySetpoint(IDLE);
	}

	@Override
	public void outputTelemetry() {
		super.outputTelemetry();
		stage2Publisher.set(ElevatorConstants.stage2Offset.plus(new Transform3d(
				new Translation3d(
						0.0,
						0.0,
						ElevatorConstants.converter.toDistance(getPosition()).in(Units.Meters)),
				new Rotation3d(0.0, 0.0, 0.0))));
		stage1Publisher.set(ElevatorConstants.stage1Offset.plus(new Transform3d(
				new Translation3d(
						0.0,
						0.0,
						ElevatorConstants.converter
								.toDistance(getPosition())
								.div(2.0)
								.in(Units.Meters)),
				new Rotation3d(0.0, 0.0, 0.0))));
	}
}
