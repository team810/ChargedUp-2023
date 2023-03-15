package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

import java.util.function.Supplier;


public class ArmCommand extends CommandBase {
	private final Arm arm;
	private final Supplier<Integer> m_angle;

	public ArmCommand(Arm arm, Supplier<Integer> angle) {
		this.arm = arm;
		m_angle = angle;


		addRequirements(this.arm);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		int aDouble = m_angle.get();
		if (aDouble == 0) {
			arm.setExtenderSetpoint(arm.getExtenderSetpoint() - .2);
		} else if (aDouble == 90) {
			arm.setPivotSetpoint(arm.getPivotSetpoint() + .2);
		} else if (aDouble == 180) {
			arm.setExtenderSetpoint(arm.getExtenderSetpoint() + .5);
		} else if (aDouble == 270) {
			arm.setPivotSetpoint(arm.getPivotSetpoint() - .5);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
