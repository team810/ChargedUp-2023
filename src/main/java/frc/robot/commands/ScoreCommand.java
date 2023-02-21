package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Autos;
import frc.robot.subsystems.*;

public class ScoreCommand extends SequentialCommandGroup {
    private final Arm arm;
    private final Drivetrain drivetrain;
    private final Gripper gripper;
    private final Limelight limelight;
    private final Conveyor conveyor;
    private final SquareToTargetCommand toTarget;
    private int targetNum;
    private int gamePiece;


    public ScoreCommand(Arm arm, Drivetrain drivetrain, Gripper gripper, Limelight limelight,
                        Conveyor conveyor, Autos auto) {
        this.arm = arm;
        this.drivetrain = drivetrain;
        this.gripper = gripper;
        this.limelight = limelight;
        this.conveyor = conveyor;
        this.toTarget = new SquareToTargetCommand(drivetrain, limelight, () -> gamePiece, auto); // FIXME wtf is the object

        addRequirements(this.arm, this.drivetrain, this.gripper, this.limelight, this.conveyor);

        addCommands(
                getGamePiece(),
                toTarget,
                armCommand(),
                new InstantCommand(gripper::rest),
                new InstantCommand(() -> System.out.println("Piece Placed")),
                new InstantCommand(arm::restPivot)
        );
    }

    private SequentialCommandGroup getGamePiece() // this is getting the game pice to the gripper from the conveyer belit
    {
        return new SequentialCommandGroup(
                new InstantCommand(() -> conveyor.setDisabled(false)),
                new WaitUntilCommand(() -> conveyor.getGamePiece() != 0),
                new InstantCommand(() ->
                {
                    gamePiece = conveyor.getGamePiece();
                    switch (conveyor.getGamePiece())
                    {
                        case 1:
                            gripper.gripCone();
                            System.out.println("Gripping something");
                            break;
                        case 2:
                            gripper.gripCube();
                            System.out.println("Gripping something");
                            break;
                        case 0:
                            System.out.println("LIFE IS A LIE");
                            break;
                    }
                }),
                new InstantCommand(() -> conveyor.setDisabled(true))

        );
    }

    private SequentialCommandGroup armCommand()
    {
        return new SequentialCommandGroup(
                new InstantCommand(() ->
                {
                    if (gamePiece == 1) // if game piece is a cone it will
                    {
                        switch (targetNum)
                        {
                            case 1:
                                arm.lowGoalCone();
                                break;
                            case 2:
                                arm.middleGoalCone();
                                break;
                            case 3:
                                arm.highGoalCone();
                                break;
                            default:
                                System.out.println("Probably Bad range for cone ");
                                break;

                        }
                    } else if (gamePiece == 2) // if the game piece is going to be a cube
                    {
                        switch (targetNum)
                        {
                            case 1:
                                arm.lowGoalCube();
                                break;
                            case 2:
                                arm.middleGoalCube();
                                break;
                            case 3:
                                arm.highGoalCube();
                                break;
                            default:
                                System.out.println("out of range error for cube");
                                break;
                        }
                    }else{
                        System.out.println("VERY BAD NOTHING IN THE GRIPPER, in code at least");
                    }
                }
                ),
                new WaitCommand(1)
        );
    }

}