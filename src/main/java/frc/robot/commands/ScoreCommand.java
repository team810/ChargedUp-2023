package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

public class ScoreCommand extends SequentialCommandGroup {
	private final Arm arm;
	private final Drivetrain drivetrain;
	private final Gripper gripper;
	private final Limelight limelight;
	private final Conveyor conveyor;
	 private final ToTargetCommand toTarget;
	private final int target;
	private int gamePiece;

	public ScoreCommand(Arm arm, Drivetrain drivetrain, Gripper gripper, Limelight limelight,
	                    Conveyor conveyor, Intake intake, int target, int targetGrid) {
		this.arm = arm;
		this.drivetrain = drivetrain;
		this.gripper = gripper;
		this.limelight = limelight;
		this.conveyor = conveyor;

		//TODO: Uncomment for squaring to target on ScoreCommand
		 this.toTarget = new ToTargetCommand(conveyor, drivetrain, limelight);

		this.target = target;
		addCommands(
				new InstantCommand(() -> arm.setExtenderSetpoint(-.5)),
				 toTarget,
//				new InstantCommand(() -> {
//					gamePiece = conveyor.getGamePiece();
//				}), // this sets the game piece after the conveyor runs
//				new InstantCommand(() -> {
//					gripper.openGripper(false);
//					gripper.gripPiece(true);
//				}),

				new InstantCommand(() -> gripper.setMotor(.4)),
				new WaitCommand(.25),
				new InstantCommand(() -> arm.setExtenderSetpoint(-3.5)),
				new RaiseArmCommand(arm, target, targetGrid),
				new WaitCommand(1.1),
				new InstantCommand(() -> gripper.setMotor(-.1)),
				new WaitCommand(.2),
				new InstantCommand(() -> gripper.setMotor(0)),
				new WaitCommand(.5),
				new InstantCommand(arm::restExtender),
				new WaitCommand(.8),
				new InstantCommand(arm::restPivot),
				new WaitCommand(2));

		addRequirements(this.arm, this.drivetrain, this.gripper, this.limelight, this.conveyor);
	}
}
