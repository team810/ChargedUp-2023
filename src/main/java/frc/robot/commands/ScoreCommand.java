package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Limelight;

public class ScoreCommand extends SequentialCommandGroup {
    private final Arm arm;
    private final ColorSensor colorSensor;
    private final Drivetrain drivetrain;
    private final Gripper gripper;
    private final Limelight limelight;
    private final Conveyor conveyor;
    private SquareToTargetCommand toTarget;
    private int targetNum;


    public ScoreCommand(Arm arm, ColorSensor colorSensor, Drivetrain drivetrain, Gripper gripper, Limelight limelight,
            Conveyor conveyor, int target[]) {
        this.arm = arm;
        this.colorSensor = colorSensor;
        this.drivetrain = drivetrain;
        this.gripper = gripper;
        this.limelight = limelight;
        this.conveyor = conveyor;
        this.toTarget = new SquareToTargetCommand(drivetrain, limelight, 0); // FIXME wtf is the object

        addRequirements(this.arm, this.colorSensor, this.drivetrain, this.gripper, this.limelight, this.conveyor);

        addCommands(
                toTarget, // This command squares the bot to the target
                armCommand(),
                new InstantCommand(gripper::rest),
                new InstantCommand(() -> System.out.println("Piece Placed")),
                new InstantCommand(arm::rest)
        );
    }

    public InstantCommand genTarget()
    {
        return new InstantCommand(
                () -> targetNum = toTarget.getTarget()
        );
    }

    private SequentialCommandGroup armCommand()
    {
        return new SequentialCommandGroup(
                new InstantCommand(() ->
                {
                    switch (targetNum)
                    {
                        case 1:
                            arm.lowGoal();
                            break;
                        case 2:
                            arm.middleGoal();
                            break;
                        case 3:
                            arm.highGoal();
                            break;
                        default:
                            System.out.println("Probably Bad range");
                            break;

                    }
                }
                ),
                new WaitCommand(1)
        );
    }

}