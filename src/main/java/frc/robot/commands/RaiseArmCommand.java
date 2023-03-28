package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;

public class RaiseArmCommand extends SequentialCommandGroup {
	private final Arm arm;
	private final int target;

	private final double reading_that_they_like = -4.3;

	public RaiseArmCommand(Arm arm, int target, int targetGrid) {

		this.arm = arm;
		this.target = target;

		if (targetGrid == 1) // target grid is for the cone
		{
			addCommands(
					new InstantCommand(() -> arm.setExtenderSetpoint(reading_that_they_like)),
					new WaitCommand(0.5),
					armToConeGoal(),
					new WaitCommand(0.75),
					extenderToConeGoal()
			);
		} else if (targetGrid == 2) { // target grid is for the cube
			addCommands(
					new InstantCommand(() -> arm.setExtenderSetpoint(reading_that_they_like)),
					new WaitCommand(0.5),
					armToCubeGoal(),
					new WaitCommand(0.75),
					extenderToCubeGoal()
			);
		}
	}

	private Command armToConeGoal() {
		return new InstantCommand(() -> {
			switch (target) {
				case 1:
					arm.setPivotSetpoint(-9.5);
					break;
				case 2:
					arm.setPivotSetpoint(-35.5);
					break;
				case 3:
					arm.setPivotSetpoint(-40);
					break;
				default:
					System.out.println("how did you get here");
					break;
			}
		});
	}

	private Command armToCubeGoal() {
		// FIXME cube constants
		return new InstantCommand(() -> {
			switch (target) {
				case 1:
					arm.setPivotSetpoint(-9.5);
					break;
				case 2:
					arm.setPivotSetpoint(-35.5);
					break;
				case 3:
					arm.setPivotSetpoint(-40);
					break;
				default:
					System.out.println("how did you get here");
					break;
			}
		});
	}

	private Command extenderToConeGoal() {
		return new InstantCommand(() ->
		{
			switch (target) {
				case 1:
					arm.setExtenderSetpoint(-2.5);
					break;
				case 2:
					arm.setExtenderSetpoint(2);
					break;
				case 3:
					arm.setExtenderSetpoint(20.5);
					break;
				default:
					System.out.println("how did you get here");
					break;
			}
		});
	}

	public Command extenderToCubeGoal() {
		// FIXME cube extender constants
		return new InstantCommand(() ->
		{
			switch (target) {
				case 1:
					arm.setExtenderSetpoint(-2.5);
					break;
				case 2:
					arm.setExtenderSetpoint(5);
					break;
				case 3:
					arm.setExtenderSetpoint(20.5);
					break;
				default:
					System.out.println("how did you get here");
					break;
			}
		});
	}
}