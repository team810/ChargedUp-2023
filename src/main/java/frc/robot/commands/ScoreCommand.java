package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class ScoreCommand extends SequentialCommandGroup {
	private final Arm arm;
	private final Drivetrain drivetrain;
	private final Gripper gripper;
	private final Conveyor conveyor;
	// private final ToTargetCommand toTarget;
	private final int target;
	private final HardStopSubsystem m_hardStop;
	private int gamePiece;

	public ScoreCommand(Arm arm, Drivetrain drivetrain, Gripper gripper,
	                    Conveyor conveyor, Intake intake, int target, int targetGrid, HardStopSubsystem mHardStop) {
		this.arm = arm;
		this.drivetrain = drivetrain;
		this.gripper = gripper;
		this.conveyor = conveyor;

		//TODO: Uncomment for squaring to target on ScoreCommand
		//  this.toTarget = new ToTargetCommand(conveyor, drivetrain, limelight);

		this.target = target;
		m_hardStop = mHardStop;
		addCommands(
				new InstantCommand(m_hardStop::out),
				new WaitCommand(0.5),
				new InstantCommand(() -> arm.setExtenderSetpoint(-1.7)),
				//  toTarget,
				new InstantCommand(() -> gripper.setMotor(.4)),
				new WaitCommand(.25),
				new InstantCommand(() -> arm.setExtenderSetpoint(-3.5)),
				new RaiseArmCommand(arm, target, targetGrid),
				new WaitCommand(1.1),
				new InstantCommand(() -> gripper.setMotor(-.1)),
				new WaitCommand(.1),
				new InstantCommand(() -> gripper.setMotor(0)),
				new WaitCommand(.5),
				new InstantCommand(() -> gripper.setMotor(0.2)), 
				new WaitCommand(0.5), 
				new InstantCommand(() -> gripper.setMotor(0)), 
				new WaitCommand(0.5),
				new InstantCommand(() -> arm.setExtenderSetpoint(-3.5)),
				new WaitCommand(.8),
				new InstantCommand(arm::restPivot),
				new WaitCommand(2));

		addRequirements(this.arm, this.drivetrain, this.gripper, this.conveyor);
	}
}
