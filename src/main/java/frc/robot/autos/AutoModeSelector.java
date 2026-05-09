package frc.robot.autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.AutoConstants.AutoEndBehavior;

public class AutoModeSelector {
	private AutoChooser mAutoChooser = new AutoChooser();

	public AutoModeSelector(AutoFactory factory) {
		SmartDashboard.putData(mAutoChooser);
	}

	public Command getSelectedCommand() {
		return mAutoChooser.selectedCommandScheduler();
	}

	public AutoChooser getAutoChooser() {
		return mAutoChooser;
	}
}
