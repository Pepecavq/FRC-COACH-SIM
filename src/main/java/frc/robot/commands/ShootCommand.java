package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.Superstructure;

public class ShootCommand extends Command {
	private final Supplier<Rotation2d> rotationSupplier;

	public ShootCommand(Supplier<Rotation2d> rotationSupplier) {
		this.rotationSupplier = rotationSupplier;
	}

	public boolean shootReady() {
		return DriveConstants.inTolerance();
	}

	@Override
	public void initialize() {
		DriveConstants.setSupplierDirection(rotationSupplier);
	}

	@Override
	public void execute() {
		if (Robot.isSimulation()) {
			if (!shootReady()) return;
			Superstructure.mInstance.SimCalcShoot();
		}
	}

	@Override
	public void end(boolean interrupted) {
		DriveConstants.setSupplierDirection(null);
		if (Robot.isSimulation()) {
			Superstructure.mInstance.lastShootTime = 0;
		}
	}
}
