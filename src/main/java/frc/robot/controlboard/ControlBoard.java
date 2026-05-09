package frc.robot.controlboard;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.FieldLayout.Level;
import frc.robot.commands.AutoIntakeFuelCommand;
import frc.robot.commands.AutoalignThenShootCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.function.Supplier;

public class ControlBoard extends SubsystemBase {
	public static final ControlBoard mInstance = new ControlBoard();

	private CommandXboxController driver = ControlBoardConstants.mDriverController;
	private CommandXboxController operator = ControlBoardConstants.mOperatorController;

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Trigger overrideTrigger = driver.rightTrigger(0.1);

	private Trigger rightBumper = driver.rightBumper();
	private Trigger endEffectorTrigger;

	public void configureBindings() {
		Drive.mInstance.setDefaultCommand(Drive.mInstance.followSwerveRequestCommand(
				DriveConstants.teleopRequest, DriveConstants.teleopRequestUpdater));
		driver.back()
				.onTrue(Commands.runOnce(
								() -> Drive.mInstance.getGeneratedDrive().seedFieldCentric(), Drive.mInstance)
						.ignoringDisable(true));

		driverControls();
		debugControls();
	}

	public void driverControls() {
		Superstructure s = Superstructure.mInstance;

		// SHOOT WHILE MOVE (right trigger) ######################################################
		driver.rightTrigger(0.1).whileTrue(
				new AutoalignThenShootCommand(s.hubSupplier));

		// AUTO INTAKE FUEL (left trigger) #######################################################
		driver.leftTrigger(0.1).whileTrue(new AutoIntakeFuelCommand());

		// AUTO PASS TRENCH (A button) ###########################################################
		driver.a().whileTrue(s.autoPassTrench());
	}


	private void debugControls() {
		Superstructure s = Superstructure.mInstance;

		
	}

	public Command rumbleCommand(Time duration) {
		return Commands.sequence(
						Commands.runOnce(() -> {
							setRumble(true);
						}),
						Commands.waitSeconds(duration.in(Units.Seconds)),
						Commands.runOnce(() -> {
							setRumble(false);
						}))
				.handleInterrupt(() -> {
					setRumble(false);
					;
				});
	}

	public void setRumble(boolean on) {
		ControlBoardConstants.mDriverController.getHID().setRumble(RumbleType.kBothRumble, on ? 1.0 : 0.0);
	}

	public void configureSysIDTests() {
		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		driver.back()
				.and(driver.y())
				.whileTrue(Drive.mInstance.getGeneratedDrive().sysIdDynamic(Direction.kForward));
		driver.back()
				.and(driver.x())
				.whileTrue(Drive.mInstance.getGeneratedDrive().sysIdDynamic(Direction.kReverse));
		driver.start()
				.and(driver.y())
				.whileTrue(Drive.mInstance.getGeneratedDrive().sysIdQuasistatic(Direction.kForward));
		driver.start()
				.and(driver.x())
				.whileTrue(Drive.mInstance.getGeneratedDrive().sysIdQuasistatic(Direction.kReverse));

		// Reset the field-centric heading on left bumper press
		driver.leftBumper().onTrue(Drive.mInstance.getGeneratedDrive().runOnce(() -> Drive.mInstance
				.getGeneratedDrive()
				.seedFieldCentric()));
	}

	public void configureModulePointing() {
		driver.a().whileTrue(Drive.mInstance.getGeneratedDrive().applyRequest(() -> brake));
		driver.b()
				.whileTrue(Drive.mInstance
						.getGeneratedDrive()
						.applyRequest(() ->
								point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));
	}
}
