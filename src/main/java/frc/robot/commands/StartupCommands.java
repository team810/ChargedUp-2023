package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Gripper;

public class StartupCommands extends ParallelCommandGroup {
	private final Gripper gripper;

	public StartupCommands(Gripper gripper) {

		this.gripper = gripper;

		addCommands(gripperCommand());
		addRequirements(gripper);
	}

	public Command gripperCommand() {
		return new SequentialCommandGroup( // This is so the gripper does not get caught on the conveyor belt.
				new InstantCommand(gripper::closeGripper),
				new WaitCommand(.5),
				new InstantCommand(gripper::openGripper));
	}

}