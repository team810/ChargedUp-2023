package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class ScoreCommand extends SequentialCommandGroup {
    private final Arm arm;
    private final Drivetrain drivetrain;
    private final Gripper gripper;
    private final Limelight limelight;
    private final Conveyor conveyor;
    private int target;
    private final ToTargetCommand toTarget;
    private int gamePiece;


    public ScoreCommand(Arm arm, Drivetrain drivetrain, Gripper gripper, Limelight limelight,
                        Conveyor conveyor, int target) {
        this.arm = arm;
        this.drivetrain = drivetrain;
        this.gripper = gripper;
        this.limelight = limelight;
        this.conveyor = conveyor;

        this.toTarget = new ToTargetCommand(conveyor, drivetrain, limelight);

        this.target = target;
        addCommands(
                new InstantCommand(() -> arm.setExtenderSetpoint(-1)),
                toTarget,
                new InstantCommand(gripper::openGripper),
                new ConveyorCommand(conveyor), // This is the command so the conveyor positions the game piece.
                new InstantCommand(() -> {gamePiece = conveyor.getGamePiece();}), // this sets the game piece after the conveyor runs
                gripGamePiece(),
                new WaitCommand(.25),
                new InstantCommand(() -> arm.setExtenderSetpoint(-3.5)),
                armToGoal(), // I think that I can increase the arm speed to make the scoring procces faster
                new WaitCommand(1.5),
                extenderToGoal(),
                new WaitCommand(1.1),
                releaseGrip(),
                new WaitCommand(.5),
                new InstantCommand(arm::restExtender),
                new InstantCommand(gripper::closeGripper),
                new WaitCommand(.8),
                new InstantCommand(arm::restPivot),
                new WaitCommand(2),
                new InstantCommand(gripper::openGripper)
        );
        addRequirements(this.arm, this.drivetrain, this.gripper, this.limelight, this.conveyor);
    }


    private Command setTarget() {
        return new InstantCommand(() -> {
            if (conveyor.getGamePiece() == 1) { // cone
//                target = 3;
                System.out.println("Hello World\n");
            } else if (conveyor.getGamePiece() == 2) { // cube
                target = 1;
            } else {
                System.out.println("NO target");
            }
        });
    }

    private Command gripGamePiece()
    {
        return new InstantCommand(() ->
        {
            if (conveyor.getGamePiece() == 1)
            {
                gripper.gripCone();
            } else if (conveyor.getGamePiece() == 2) {
                gripper.gripCube();
            }else{
                gripper.gripCube(); // FIXME this is not a good default state
            }
        });
    }

    public Command releaseGrip()
    {
        return new InstantCommand(gripper::openGripper);
    }

    private Command armToConeGoal()
    {
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

    private Command armToCubeGoal()
    {
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


    private Command armToGoal()
    {
        if (gamePiece == 1)
        {
            return armToConeGoal();
        } else if (gamePiece == 2) {
            return armToCubeGoal();
        }else{
            return armToConeGoal();
        }
    }

    private Command extenderToConeGoal()
    {
        return new InstantCommand(() ->
        {
            switch (target)
            {
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

    public Command extenderToCubeGoal()
    {
        // FIXME cube extender constants
        return new InstantCommand(() ->
        {
            switch (target)
            {
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

    private Command extenderToGoal()
    {
        if (gamePiece == 1)
        {
            return extenderToConeGoal();
        } else if (gamePiece == 2) {
            return extenderToCubeGoal();
        }else{
            return extenderToConeGoal();
        }
    }

}
