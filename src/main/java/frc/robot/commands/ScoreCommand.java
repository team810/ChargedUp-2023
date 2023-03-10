package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

public class ScoreCommand extends SequentialCommandGroup {
	private final Arm arm;
	private final Drivetrain drivetrain;
	private final Gripper gripper;
	private final Limelight limelight;
	private final Conveyor conveyor;
	// private final ToTargetCommand toTarget;
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
		// this.toTarget = new ToTargetCommand(conveyor, drivetrain, limelight);

		this.target = target;
		addCommands(
				new InstantCommand(() -> arm.setExtenderSetpoint(-1)),
				// toTarget,
				new InstantCommand(() -> {
					gamePiece = conveyor.getGamePiece();
				}), // this sets the game piece after the conveyor runs
				new InstantCommand(() -> {
					gripper.openGripper(false);
					gripper.gripPiece(true);
				}),
				new WaitCommand(.25),
				new InstantCommand(() -> arm.setExtenderSetpoint(-3.5)),
				new RaiseArmCommand(arm, target, targetGrid),
				new WaitCommand(1.1),
				new InstantCommand(() -> {
					gripper.openGripper(true);
					gripper.gripPiece(false);
				}),
				new WaitCommand(.5),
				new InstantCommand(arm::restExtender),
				new WaitCommand(.8),
				new InstantCommand(arm::restPivot),
				new WaitCommand(2));

		addRequirements(this.arm, this.drivetrain, this.gripper, this.limelight, this.conveyor);
	}
}
