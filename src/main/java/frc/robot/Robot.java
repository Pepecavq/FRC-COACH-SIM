// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.io.DetectionIOLimelight.DetectionMode;
import frc.lib.io.MotorIO.Setpoint;
import frc.lib.logging.LogUtil.GcStatsCollector;
import frc.lib.logging.LoggedTracer;
import frc.lib.util.Stopwatch;
import frc.robot.autos.AutoHelpers;
import frc.robot.autos.AutoModeSelector;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.detection.Detection;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Limelight;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.frc2026.Mock.Logger;

public class Robot extends TimedRobot {
	private Command mAutonomousCommand;
	private AutoModeSelector mAutoModeSelector;
	private GcStatsCollector mGcStatsCollector = new GcStatsCollector();

	private static String mPreviousAutoName;
	private static String kSerial;
	private Stopwatch buttonWait = new Stopwatch();

	public static final Stopwatch autoTimer = new Stopwatch();

	private long disabledLoopCount = 0;

	public Robot() {
		if (Robot.isSimulation()) {
			SimulatedArena.overrideInstance(new Arena2026Rebuilt(true));
			if(true){
				SimulatedArena.getInstance().resetFieldForAuto();
			}
		}
		RobotConstants.mAutoFactory = new AutoFactory(
				Drive.mInstance::getPose,
				Drive.mInstance.getGeneratedDrive()::resetPose,
				Drive.mInstance::followChoreoTrajectory,
				true,
				Drive.mInstance);

		AutoHelpers.bindEventMarkers(RobotConstants.mAutoFactory);

		mAutoModeSelector = new AutoModeSelector(RobotConstants.mAutoFactory);
		mPreviousAutoName = mAutoModeSelector.getSelectedCommand().getName();

		RobotModeTriggers.autonomous()
				.onFalse(Commands.runOnce(() -> Drive.mInstance.setSwerveRequest(new SwerveRequest.ApplyFieldSpeeds()))
						.ignoringDisable(true));

		Map<String, Integer> commandCounts = new HashMap<>();
		BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
			String name = command.getName();
			int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
			commandCounts.put(name, count);
			SmartDashboard.putBoolean(
					"Commands/Unique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
			SmartDashboard.putBoolean("Commands/All/" + name, count > 0);
		};

		SmartDashboard.putBoolean("Is Comp", RobotConstants.isComp);
		SmartDashboard.putBoolean("Is Omega", RobotConstants.isOmega);

		CommandScheduler.getInstance()
				.onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
		CommandScheduler.getInstance().onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
		CommandScheduler.getInstance()
				.onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));
	}

	@Override
	public void robotInit() {
		ControlBoard.mInstance.configureBindings();

		for (SubsystemBase s : new SubsystemBase[] {
			Drive.mInstance,
			Intake.mInstance,
			Limelight.mInstance,
			Pivot.mInstance,
			Superstructure.mInstance
		}) {
			SmartDashboard.putData(s);
		}

		SmartDashboard.putData("Auto Mode Selector", mAutoModeSelector.getAutoChooser());
		SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

		RobotController.setBrownoutVoltage(Units.Volts.of(4.6));

		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());

		Limelight.mInstance.disable(false);

		CommandScheduler.getInstance().setPeriod(0.02);
	}

	@Override
	public void robotPeriodic() {
		LoggedTracer.reset();

		try {
			Threads.setCurrentThreadPriority(true, 4);
			double commandSchedulerStart = Timer.getTimestamp();
			CommandScheduler.getInstance().run();
			double commandSchedulerEnd = Timer.getTimestamp();
			LoggedTracer.record("Commands");
			mGcStatsCollector.update();
			SmartDashboard.putNumber(
					"Logged Robot/Loop Cycle Time Milliseconds",
					(commandSchedulerEnd - commandSchedulerStart) * 1000.0);
			Threads.setCurrentThreadPriority(false, 0);
		} catch (Exception e) {
			SmartDashboard.putString("Logged Robot/Latest Error", e.getMessage());
		}

		Logger.recordOutput("FieldFuels", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
	}

	@Override
	public void disabledInit() {
		disabledLoopCount = 0;

		buttonWait.start();
		Drive.mInstance.getGeneratedDrive().configNeutralMode(NeutralModeValue.Brake);

		Intake.mInstance.applySetpoint(Setpoint.withNeutralSetpoint());
		Pivot.mInstance.applySetpoint(Setpoint.withNeutralSetpoint());

		Detection.mInstance.setPipeline(DetectionMode.AUTO.index);
	}

	@Override
	public void disabledPeriodic() {
		disabledLoopCount++;

		if (DriverStation.getAlliance().isPresent()) {
			RobotConstants.isRedAlliance = DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
		} else {
			RobotConstants.isRedAlliance =
					DriverStationSim.getAllianceStationId().equals(AllianceStationID.Red1)
							|| DriverStationSim.getAllianceStationId().equals(AllianceStationID.Red2)
							|| DriverStationSim.getAllianceStationId().equals(AllianceStationID.Red3);
		}

		if (disabledLoopCount % 50 == 0) {
		
		}
	}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		mAutonomousCommand = mAutoModeSelector.getSelectedCommand();
		autoTimer.start();

		if (mAutonomousCommand != null) {
			mAutonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {
		mAutoModeSelector.getAutoChooser().select("Nothing");

		autoTimer.reset();
	}

	@Override
	public void teleopInit() {
		if (mAutonomousCommand != null) {
			mAutonomousCommand.cancel();
		}

		CommandScheduler.getInstance().clearComposedCommands();

		Drive.mInstance.setSwerveRequest(new SwerveRequest.ApplyFieldSpeeds());

		Detection.mInstance.setPipeline(DetectionMode.AUTO.index);
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}

	@Override
	public void simulationPeriodic() {}
}
