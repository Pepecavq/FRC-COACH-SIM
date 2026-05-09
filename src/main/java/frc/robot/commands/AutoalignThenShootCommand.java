package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants;

public class AutoalignThenShootCommand extends ShootCommand {

	public AutoalignThenShootCommand(Supplier<Rotation2d> rotationSupplier) {
		super(rotationSupplier);
	}

	@Override
	public void initialize() {
		super.initialize();
		DriveConstants.pointToPointActive = true;
	}

	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);
		DriveConstants.pointToPointActive = false;
	}
}
