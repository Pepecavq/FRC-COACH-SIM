package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.bases.ServoMotorSubsystem.ServoHomingConfig;
import frc.lib.io.MotorIOTalonFX;
import frc.lib.io.MotorIOTalonFX.MotorIOTalonFXConfig;
import frc.lib.io.MotorIOTalonFXSim;
import frc.lib.sim.LinearSim;
import frc.lib.sim.LinearSim.LinearSimConstants;
import frc.lib.util.Util;
import frc.robot.Ports;
import frc.robot.Robot;

public class IntakeConstants {

	public static final double kGearing = 1.0;

	public static final Util.DistanceAngleConverter converter = new Util.DistanceAngleConverter(
			Units.Inches.of(1.0));

	public static final Distance kMaxHeight = Units.Inches.of(12.0);
	public static final Distance kStowPosition = Units.Inches.of(0.0);
	public static final Distance kDeployPosition = Units.Inches.of(8.0);
	public static final Distance kEpsilonThreshold = Units.Inches.of(0.5);

	public static TalonFXConfiguration getFXConfig() {
		TalonFXConfiguration FXConfig = new TalonFXConfiguration();
		FXConfig.Slot0.kP = 8.0;
		FXConfig.Slot0.kD = 0.2;
		FXConfig.Slot0.kS = 0.25;

		FXConfig.MotionMagic.MotionMagicCruiseVelocity = 20.0;

		FXConfig.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
		FXConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
		FXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;

		FXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		FXConfig.CurrentLimits.StatorCurrentLimit = 60.0;

		FXConfig.Voltage.PeakForwardVoltage = 12.0;
		FXConfig.Voltage.PeakReverseVoltage = -12.0;

		FXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		FXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
				converter.toAngle(kMaxHeight).in(Units.Rotations);

		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		FXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
				converter.toAngle(kStowPosition).minus(Units.Degrees.of(10.0)).in(Units.Rotations);

		FXConfig.Feedback.SensorToMechanismRatio = kGearing;

		FXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		return FXConfig;
	}

	public static MotorIOTalonFXConfig getIOConfig() {
		MotorIOTalonFXConfig IOConfig = new MotorIOTalonFXConfig();
		IOConfig.mainConfig = getFXConfig();
		IOConfig.mainID = Ports.INTAKE_MAIN.id;
		IOConfig.mainBus = Ports.INTAKE_MAIN.bus;
		IOConfig.unit = converter.getDistanceUnitAsAngleUnit(Units.Inches);
		IOConfig.time = Units.Second;
		return IOConfig;
	}

	public static LinearSimConstants getSimConstants() {
		LinearSimConstants simConstants = new LinearSimConstants();
		simConstants.motor = DCMotor.getKrakenX60Foc(1);
		simConstants.gearing = kGearing;
		simConstants.carriageMass = Units.Pounds.of(5);
		simConstants.startingHeight = kStowPosition;
		simConstants.minHeight = kStowPosition;
		simConstants.maxHeight = kMaxHeight;
		simConstants.simGravity = false;
		simConstants.converter = converter;
		return simConstants;
	}

	public static MotorIOTalonFX getMotorIO() {
		if (Robot.isReal()) {
			return new MotorIOTalonFX(getIOConfig());
		} else {
			return new MotorIOTalonFXSim(getIOConfig(), new LinearSim(getSimConstants()));
		}
	}

	public static ServoHomingConfig getServoConfig() {
		ServoHomingConfig servoConfig = new ServoHomingConfig();
		servoConfig.kHomePosition = converter.toAngle(kStowPosition);
		servoConfig.kHomingTimeout = Units.Seconds.of(0.5);
		servoConfig.kHomingVoltage = Units.Volts.of(-0.5);
		servoConfig.kSetHomedVelocity = converter.toAngle(Units.Inches.of(0.1)).per(Units.Second);
		return servoConfig;
	}
}
